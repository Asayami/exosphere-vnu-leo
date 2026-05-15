#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/satellite-fading-external-input-trace-container.h"
#include "ns3/satellite-free-space-loss.h"
#include "ns3/satellite-id-mapper.h"
#include "ns3/satellite-env-variables.h"
#include "ns3/satellite-utils.h"
#include <ns3/dummy-traffic-module.h>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <unistd.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("sat-gw-handover-example");

namespace
{
struct LinkMetricsSample
{
    bool valid; // false if disconnected (in LogMetrics check), true if receive good signal in LinkBudgetTraceCb
    double cnDb;
    double fsplDb;
    double atmLossDb;
    double rxPowerDbW;
    double sinrDb;
    int32_t satId;
    int32_t beamId;
    SatEnums::ChannelType_t channelType;
    double lastUpdateTime; // Added to track disconnection

    LinkMetricsSample()
        : valid(false),
          cnDb(std::numeric_limits<double>::quiet_NaN()),
          fsplDb(std::numeric_limits<double>::quiet_NaN()),
          atmLossDb(std::numeric_limits<double>::quiet_NaN()),
          rxPowerDbW(std::numeric_limits<double>::quiet_NaN()),
          sinrDb(std::numeric_limits<double>::quiet_NaN()),
          satId(-1),
          beamId(-1),
          channelType(SatEnums::UNKNOWN_CH),
          lastUpdateTime(-1.0)
    {
    }
};

static std::map<uint32_t, LinkMetricsSample> g_utMetrics;
static std::map<uint32_t, LinkMetricsSample> g_gwMetrics;
static std::ofstream g_metricsStream; // CSV output stream for link metrics
static Ptr<SatFreeSpaceLoss> g_freeSpaceLoss;
static Time g_metricsInterval = MilliSeconds(500);
static bool g_enableAtmosphericLoss = false;
static std::string g_utPositionsFilePath;

// Log every 500ms as required for web dashboard
static std::map<uint32_t, Time> g_lastProcessedTime;
static Time g_throttleInterval = MilliSeconds(10);

// Global traffic byte counters per node (UT and GW) - accumulated from simulation start
static std::map<uint32_t, uint64_t> g_utRxBytes;
static std::map<uint32_t, uint64_t> g_gwRxBytes;

// FlowMonitor address maps and per-flow tracking (for rx_bytes in CSV)
static std::map<Ipv4Address, Ptr<Node>> g_utUserIpToUtNode;
static std::map<Ipv4Address, uint32_t> g_utUserIpToUtId;
static std::set<Ipv4Address> g_gwUserIps;
static std::map<FlowId, uint64_t> g_flowRxBytesLast;

// Lightweight raw data storage - populated by callback, consumed by LogMetrics
// This allows us to throttle expensive calculations while maintaining 500ms data resolution
struct RawLinkData {
    double rxPower_W;
    double ifPower;
    double sinr;
    uint32_t satId;
    uint32_t beamId;
    SatEnums::ChannelType_t channelType;
    Ptr<MobilityModel> rxMobility;
    Ptr<MobilityModel> txMobility;
    double carrierFreq_hz;
    bool valid;
    Time timestamp;
};
static std::map<uint32_t, RawLinkData> g_utRawData;
static std::map<uint32_t, RawLinkData> g_gwRawData;

// Profiling: track time spent in callbacks to identify bottlenecks
static std::chrono::duration<double> g_linkBudgetCbTime{0};
static uint64_t g_linkBudgetCbCalls = 0;
static uint64_t g_linkBudgetCbThrottled = 0;

// FlowMonitor for accurate traffic statistics
static Ptr<FlowMonitor> g_flowMonitor;
static FlowMonitorHelper g_flowHelper;

void
AddDisconnectedUtsFromFile(const std::string& utFile,
                            Ptr<SatAntennaGainPatternContainer> antennaPatterns)
{
    if (!antennaPatterns) return;

    std::ifstream input(utFile.c_str());
    if (!input.is_open())
    {
        NS_LOG_WARN("Failed to open UT positions file: " << utFile);
        return;
    }

    std::vector<GeoCoordinate> disconnected;
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    while (input >> lat >> lon >> alt)
    {
        GeoCoordinate position(lat, lon, alt);
        uint32_t satId = Singleton<SatTopology>::Get()->GetClosestSat(position);
        uint32_t bestBeamId = antennaPatterns->GetBestBeamId(satId, position, true);
        if (bestBeamId == 0) disconnected.push_back(position);
    }

    if (disconnected.empty()) return;

    NodeContainer uts;
    uts.Create(disconnected.size());
    Ptr<SatListPositionAllocator> allocator = CreateObject<SatListPositionAllocator>();
    for (const auto& position : disconnected)
    {
        allocator->Add(position);
    }

    MobilityHelper mobility;
    mobility.SetPositionAllocator(allocator);
    mobility.SetMobilityModel("ns3::SatConstantPositionMobilityModel");
    mobility.Install(uts);

    for (NodeContainer::Iterator it = uts.Begin(); it != uts.End(); ++it)
    {
        // Add disconnected UTs to topology (no device, just position)
        Singleton<SatTopology>::Get()->AddUtNode(*it);
    }
}

double
ComputeCnDb(double rxPowerW, double ifPowerW, double sinrLin)
{
    if (rxPowerW <= 0.0 || sinrLin <= 0.0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double totalIn = rxPowerW / sinrLin;
    double noiseW = totalIn - ifPowerW;
    if (noiseW <= 0.0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return SatUtils::LinearToDb(rxPowerW / noiseW);
}

bool
GetGroundReceiver(SatEnums::ChannelType_t channelType,
                  const Mac48Address& receiverAddress,
                  uint32_t& nodeId,
                  Ptr<MobilityModel>& mobility,
                  bool& isUt)
{
    if (channelType == SatEnums::FORWARD_USER_CH)
    {
        int32_t utId = Singleton<SatIdMapper>::Get()->GetUtIdWithMac(receiverAddress);
        if (utId <= 0)
        {
            return false;
        }

        nodeId = static_cast<uint32_t>(utId);
        Ptr<Node> utNode = Singleton<SatTopology>::Get()->GetUtNode(nodeId - 1);
        mobility = utNode->GetObject<MobilityModel>();
        isUt = true;
        return mobility != nullptr;
    }

    if (channelType == SatEnums::RETURN_FEEDER_CH)
    {
        int32_t gwId = Singleton<SatIdMapper>::Get()->GetGwIdWithMac(receiverAddress);
        if (gwId <= 0)
        {
            return false;
        }

        nodeId = static_cast<uint32_t>(gwId);
        Ptr<Node> gwNode = Singleton<SatTopology>::Get()->GetGwNode(nodeId - 1);
        mobility = gwNode->GetObject<MobilityModel>();
        isUt = false;
        return mobility != nullptr;
    }

    return false;
}

double
GetAtmosphericLossDb(uint32_t nodeId,
                     SatEnums::ChannelType_t channelType,
                     Ptr<MobilityModel> mobility)
{
    if (!g_enableAtmosphericLoss)
    {
        return 0.0;
    }

    Ptr<SatFadingExternalInputTrace> trace =
        Singleton<SatFadingExternalInputTraceContainer>::Get()->GetFadingTrace(nodeId,
                                                                               channelType,
                                                                               mobility);
    if (!trace)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return SatUtils::LinearToDb(trace->GetFading());
}

uint32_t
GetUtIdFromNode(Ptr<Node> utNode)
{
    if (!utNode)
    {
        return 0;
    }

    const SatIdMapper* satIdMapper = Singleton<SatIdMapper>::Get();
    const Address utMac = satIdMapper->GetUtMacWithNode(utNode);
    if (utMac.IsInvalid())
    {
        return 0;
    }

    const int32_t utId = satIdMapper->GetUtIdWithMac(utMac);
    return (utId > 0) ? static_cast<uint32_t>(utId) : 0;
}

uint32_t
GetGwIdFromNode(Ptr<Node> gwNode)
{
    if (!gwNode)
    {
        return 0;
    }

    const SatIdMapper* satIdMapper = Singleton<SatIdMapper>::Get();
    const Address gwMac = satIdMapper->GetGwMacWithNode(gwNode);
    if (gwMac.IsInvalid())
    {
        return 0;
    }

    const int32_t gwId = satIdMapper->GetGwIdWithMac(gwMac);
    return (gwId > 0) ? static_cast<uint32_t>(gwId) : 0;
}

void
BuildUserIpMaps()
{
    g_utUserIpToUtNode.clear();
    g_utUserIpToUtId.clear();
    g_gwUserIps.clear();

    NodeContainer utUsers = Singleton<SatTopology>::Get()->GetUtUserNodes();
    for (NodeContainer::Iterator it = utUsers.Begin(); it != utUsers.End(); ++it)
    {
        Ptr<Node> utUser = *it;
        Ptr<Ipv4> ipv4 = utUser->GetObject<Ipv4>();
        if (!ipv4 || ipv4->GetNInterfaces() == 0)
        {
            continue;
        }

        Ptr<Node> utNode = Singleton<SatTopology>::Get()->GetUtNode(utUser);
        if (!utNode)
        {
            continue;
        }

        const uint32_t utId = GetUtIdFromNode(utNode);
        if (utId == 0)
        {
            continue;
        }

        const uint32_t ifIndex = (ipv4->GetNInterfaces() > 1) ? 1 : 0;
        for (uint32_t i = 0; i < ipv4->GetNAddresses(ifIndex); ++i)
        {
            const Ipv4Address addr = ipv4->GetAddress(ifIndex, i).GetLocal();
            if (addr == Ipv4Address::GetLoopback() || addr == Ipv4Address::GetZero())
            {
                continue;
            }
            g_utUserIpToUtNode[addr] = utNode;
            g_utUserIpToUtId[addr] = utId;
        }
    }

    NodeContainer gwUsers = Singleton<SatTopology>::Get()->GetGwUserNodes();
    for (NodeContainer::Iterator it = gwUsers.Begin(); it != gwUsers.End(); ++it)
    {
        Ptr<Node> gwUser = *it;
        Ptr<Ipv4> ipv4 = gwUser->GetObject<Ipv4>();
        if (!ipv4 || ipv4->GetNInterfaces() == 0)
        {
            continue;
        }

        const uint32_t ifIndex = (ipv4->GetNInterfaces() > 1) ? 1 : 0;
        for (uint32_t i = 0; i < ipv4->GetNAddresses(ifIndex); ++i)
        {
            const Ipv4Address addr = ipv4->GetAddress(ifIndex, i).GetLocal();
            if (addr == Ipv4Address::GetLoopback() || addr == Ipv4Address::GetZero())
            {
                continue;
            }
            g_gwUserIps.insert(addr);
        }
    }
}

void
UpdateFlowMonitorBytes()
{
    if (!g_flowMonitor)
    {
        return;
    }

    if (g_utUserIpToUtId.empty() && g_gwUserIps.empty())
    {
        BuildUserIpMaps();
    }

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(g_flowHelper.GetClassifier());
    if (!classifier)
    {
        return;
    }

    std::map<FlowId, FlowMonitor::FlowStats> stats = g_flowMonitor->GetFlowStats();
    for (const auto& flow : stats)
    {
        const FlowId flowId = flow.first;
        const uint64_t rxBytes = flow.second.rxBytes;

        uint64_t lastBytes = 0;
        auto lastIt = g_flowRxBytesLast.find(flowId);
        if (lastIt != g_flowRxBytesLast.end())
        {
            lastBytes = lastIt->second;
        }
        const uint64_t delta = (rxBytes >= lastBytes) ? (rxBytes - lastBytes) : rxBytes;
        g_flowRxBytesLast[flowId] = rxBytes;

        if (delta == 0)
        {
            continue;
        }

        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);

        auto utIt = g_utUserIpToUtId.find(t.destinationAddress);
        if (utIt != g_utUserIpToUtId.end())
        {
            g_utRxBytes[utIt->second] += delta;
            continue;
        }

        if (g_gwUserIps.find(t.destinationAddress) != g_gwUserIps.end())
        {
            auto srcIt = g_utUserIpToUtNode.find(t.sourceAddress);
            if (srcIt != g_utUserIpToUtNode.end())
            {
                Ptr<Node> gwNode = Singleton<SatTopology>::Get()->GetGwFromUt(srcIt->second);
                if (gwNode)
                {
                    const uint32_t gwId = GetGwIdFromNode(gwNode);
                    if (gwId > 0)
                    {
                        g_gwRxBytes[gwId] += delta;
                    }
                }
            }
        }
    }
}


void
LinkBudgetTraceCb(std::string path,
                  Ptr<SatSignalParameters> params,
                  Mac48Address receiverAddress,
                  Mac48Address,
                  double ifPower,
                  double sinr)
{
    auto startCb = std::chrono::high_resolution_clock::now();
    g_linkBudgetCbCalls++;

    // Bỏ qua nếu tham số rỗng
    if (!params || !g_freeSpaceLoss) return;

    // [FIX CRASH]: Kiểm tra con trỏ trạm phát
    if (!params->m_phyTx) return;

    uint32_t nodeId = 0;
    bool isUt = false;
    Ptr<MobilityModel> rxMobility;
    if (!GetGroundReceiver(params->m_channelType, receiverAddress, nodeId, rxMobility, isUt))
    {
        return;
    }

    // [THROTTLING]: Store raw data only once per 500ms per node
    // This prevents millions of expensive calculations while maintaining dashboard resolution
    Time now = Simulator::Now();
    auto lastIt = g_lastProcessedTime.find(nodeId);
    if (lastIt != g_lastProcessedTime.end())
    {
        if (now - lastIt->second < g_throttleInterval)
        {
            g_linkBudgetCbThrottled++;
            return; // Skip - already have recent data for this node
        }
    }
    g_lastProcessedTime[nodeId] = now;

    // ALWAYS store raw data regardless of SINR value
    // The SINR check for "soft disconnect" is removed - we want to log all states including poor signal
    // This ensures GW connections are tracked even with marginal SINR
    
    // Store raw data without expensive calculations
    // Expensive calculations (FSPL, C/N, etc.) will be done in LogMetrics
    RawLinkData raw;
    raw.rxPower_W = params->m_rxPower_W;
    raw.ifPower = ifPower;
    raw.sinr = sinr;
    raw.satId = params->m_satId;
    raw.beamId = params->m_beamId;
    raw.channelType = params->m_channelType;
    raw.rxMobility = rxMobility;
    raw.txMobility = params->m_phyTx->GetMobility();
    raw.carrierFreq_hz = params->m_carrierFreq_hz;
    raw.valid = true;
    raw.timestamp = now;

    if (isUt) g_utRawData[nodeId] = raw;
    else g_gwRawData[nodeId] = raw;

    auto endCb = std::chrono::high_resolution_clock::now();
    g_linkBudgetCbTime += (endCb - startCb);
}

void
LogMetrics()
{
    if (!g_metricsStream.is_open())
    {
        return;
    }

    const double now = Simulator::Now().GetSeconds();
    const Time currentTime = Simulator::Now();
    // Stale threshold should be larger than throttling interval to avoid marking throttled data as disconnected
    const Time staleThreshold = Seconds(2.0);

    // Update traffic byte counters from FlowMonitor (per-flow deltas)
    UpdateFlowMonitorBytes();

    // Process UT raw data: do expensive calculations here (once per 500ms, not per packet)
    for (auto& entry : g_utRawData)
    {
        const uint32_t nodeId = entry.first;
        RawLinkData& raw = entry.second;
        
        // Check if data is stale (no recent update)
        if (currentTime - raw.timestamp > staleThreshold)
        {
            // Mark as invalid/disconnected
            LinkMetricsSample sample;
            sample.valid = false;
            sample.satId = -1;
            sample.beamId = -1;
            sample.channelType = SatEnums::UNKNOWN_CH;
            sample.lastUpdateTime = raw.timestamp.GetSeconds();
            g_utMetrics[nodeId] = sample;
            continue;
        }
        
        // Calculate SINR in dB - ALWAYS record the value regardless of quality
        // Poor SINR is still valid data showing link degradation, not "disconnected"
        double sinrDbValue = SatUtils::LinearToDb(raw.sinr);
        
        // [EXPENSIVE CALCULATIONS]: Only done once per 500ms, not per packet
        double fsplDb = g_freeSpaceLoss->GetFsldB(raw.txMobility, raw.rxMobility, raw.carrierFreq_hz);
        double atmLossDb = GetAtmosphericLossDb(nodeId, raw.channelType, raw.rxMobility);
        
        LinkMetricsSample sample;
        sample.valid = true;
        sample.cnDb = ComputeCnDb(raw.rxPower_W, raw.ifPower, raw.sinr);
        sample.fsplDb = fsplDb;
        sample.atmLossDb = atmLossDb;
        sample.rxPowerDbW = SatUtils::WToDbW(raw.rxPower_W);
        sample.sinrDb = sinrDbValue;
        sample.satId = raw.satId;
        sample.beamId = raw.beamId;
        sample.channelType = raw.channelType;
        sample.lastUpdateTime = now;
        
        g_utMetrics[nodeId] = sample;
    }

    // Process GW raw data
    for (auto& entry : g_gwRawData)
    {
        const uint32_t nodeId = entry.first;
        RawLinkData& raw = entry.second;
        
        if (currentTime - raw.timestamp > staleThreshold)
        {
            LinkMetricsSample sample;
            sample.valid = false;
            sample.satId = -1;
            sample.beamId = -1;
            sample.channelType = SatEnums::UNKNOWN_CH;
            sample.lastUpdateTime = raw.timestamp.GetSeconds();
            g_gwMetrics[nodeId] = sample;
            continue;
        }
        
        // Calculate SINR in dB - ALWAYS record the value regardless of quality
        double sinrDbValue = SatUtils::LinearToDb(raw.sinr);
        
        double fsplDb = g_freeSpaceLoss->GetFsldB(raw.txMobility, raw.rxMobility, raw.carrierFreq_hz);
        double atmLossDb = GetAtmosphericLossDb(nodeId, raw.channelType, raw.rxMobility);
        
        LinkMetricsSample sample;
        sample.valid = true;
        sample.cnDb = ComputeCnDb(raw.rxPower_W, raw.ifPower, raw.sinr);
        sample.fsplDb = fsplDb;
        sample.atmLossDb = atmLossDb;
        sample.rxPowerDbW = SatUtils::WToDbW(raw.rxPower_W);
        sample.sinrDb = sinrDbValue;
        sample.satId = raw.satId;
        sample.beamId = raw.beamId;
        sample.channelType = raw.channelType;
        sample.lastUpdateTime = now;
        
        g_gwMetrics[nodeId] = sample;
    }

    // Log UT metrics with handover info and loss indicators
    for (const auto& entry : g_utMetrics)
    {
        const uint32_t nodeId = entry.first;
        const LinkMetricsSample& sample = entry.second;
        const std::string channelName =
            sample.valid ? SatEnums::GetChannelTypeName(sample.channelType) : "NA";
        const int32_t satIdOut = sample.valid ? sample.satId : -1;
        const int32_t beamIdOut = sample.valid ? sample.beamId : -1;
        const uint64_t rxBytes = g_utRxBytes[nodeId];

        g_metricsStream << now << ",UT," << nodeId << "," << channelName << "," << satIdOut
                        << "," << beamIdOut << "," << sample.cnDb << "," << sample.fsplDb
                        << "," << sample.atmLossDb << "," << sample.rxPowerDbW << ","
                        << sample.sinrDb << "," << rxBytes << "\n";
    }

    // Log GW metrics
    for (const auto& entry : g_gwMetrics)
    {
        const uint32_t nodeId = entry.first;
        const LinkMetricsSample& sample = entry.second;
        const std::string channelName =
            sample.valid ? SatEnums::GetChannelTypeName(sample.channelType) : "NA";
        const int32_t satIdOut = sample.valid ? sample.satId : -1;
        const int32_t beamIdOut = sample.valid ? sample.beamId : -1;
        const uint64_t rxBytes = g_gwRxBytes[nodeId];

        g_metricsStream << now << ",GW," << nodeId << "," << channelName << "," << satIdOut
                        << "," << beamIdOut << "," << sample.cnDb << "," << sample.fsplDb
                        << "," << sample.atmLossDb << "," << sample.rxPowerDbW << ","
                        << sample.sinrDb << "," << rxBytes << "\n";
    }

    // Flush every 500ms to ensure data is available for web dashboard
    g_metricsStream.flush();

    Simulator::Schedule(g_metricsInterval, &LogMetrics);
}

void
InitMetricsLog()
{
    if (g_metricsStream.is_open())
    {
        return;
    }

    char cwd[1024];
    std::string pwd = (getcwd(cwd, sizeof(cwd)) != NULL) ? std::string(cwd) : ".";
    std::string projectPath = pwd + "/../exosphere-vnu-leo";
    std::string filePath = projectPath + "/output/log.csv";
    
    g_metricsStream.open(filePath.c_str(), std::ios::out);
    g_metricsStream.setf(std::ios::fixed);
    // CSV Header: time, node info, handover tracking (sat/beam changes), loss indicators (SINR/C/N), traffic bytes
    g_metricsStream << std::setprecision(6)
                    << "time_s,node_type,node_id,channel_type,sat_id,beam_id,c_n_db,fspl_db,"
                       "atm_loss_db,rx_power_dbw,sinr_db,rx_bytes\n";
}

void ProgressReport()
{
    static auto start = std::chrono::steady_clock::now();
    static uint64_t lastEventAmount = 0;
    static double lastSimTime = 0.0;

    const double currentSimTime = Simulator::Now().GetSeconds();
    const uint64_t currentEventAmount = Simulator::GetEventCount();
    const auto now = std::chrono::steady_clock::now();
    const double realSec = std::chrono::duration_cast<std::chrono::duration<double>>(now - start).count();
    static double lastRealDuration = 0.0;

    const uint64_t deltaEventAmount = currentEventAmount - lastEventAmount;
    const double deltaSimTime = currentSimTime - lastSimTime;
    const double evAmtPerSec = (deltaSimTime > 0.0) ? (deltaEventAmount / deltaSimTime) : 0.0;
    const double realPerSim = (currentSimTime > 0.0) ? (realSec / currentSimTime) : 0.0;
    
    std::cout << "[PROGRESS] t=" << currentSimTime << "s real=" << std::setprecision(3) << realSec
              << "s currentEventAmount=" << lastEventAmount << "+" << deltaEventAmount
              << " ev_amt_per_sec=" << std::setprecision(3) << evAmtPerSec
              << " real_per_sim=" << std::setprecision(3) << realPerSim;

    // Show profiling data for LinkBudgetTrace callback
    double cbTimeMs = g_linkBudgetCbTime.count() * 1000.0;
    double cbTimePerCall = (g_linkBudgetCbCalls > 0) ? (cbTimeMs / g_linkBudgetCbCalls) : 0.0;

    // Calculate time spent outside our callback (ns-3 internal time)
    double deltaRealSec = (realSec - lastRealDuration);
    double internalTimeRatio = (deltaRealSec > 0.001) ? ((deltaRealSec - g_linkBudgetCbTime.count()) / deltaRealSec * 100.0) : 0.0;

    std::cout << " | LBTrace: " << g_linkBudgetCbCalls << " calls, "
              << g_linkBudgetCbThrottled << " throttled, "
              << std::setprecision(2) << cbTimeMs << "ms total, "
              << std::setprecision(4) << cbTimePerCall << "ms/call"
              << " | Internal: " << std::setprecision(1) << internalTimeRatio << "%";
    std::cout << std::endl;
    
    g_linkBudgetCbTime = std::chrono::duration<double>::zero();
    g_linkBudgetCbCalls = 0;
    g_linkBudgetCbThrottled = 0;

    lastEventAmount = currentEventAmount;
    lastSimTime = currentSimTime;
    lastRealDuration = realSec;

    // Cleanup throttle map periodically to prevent memory growth
    g_lastProcessedTime.clear();

    Simulator::Schedule(Seconds(5.0), &ProgressReport);
}

}

int main(int argc, char* argv[])
{
    double simulationTime = 100.0;
    bool enableProgressReport = true;
    bool enableFwdTraffic = true;
    bool enableRtnTraffic = true;
    bool enableLinkMetricsLog = true;
    bool enableAtmosphericLoss = g_enableAtmosphericLoss;
    Time metricsInterval = g_metricsInterval;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation length in seconds", simulationTime);
    cmd.AddValue("enableProgressReport", "Enable lightweight progress timing report",
                 enableProgressReport);
    cmd.AddValue("enableFwdTraffic", "Enable forward-link traffic (GW -> UT)", enableFwdTraffic);
    cmd.AddValue("enableRtnTraffic", "Enable return-link traffic (UT -> GW)", enableRtnTraffic);
    cmd.AddValue("enableLinkMetricsLog",
                 "Enable per-UT/GW link metrics logging",
                 enableLinkMetricsLog);
    cmd.AddValue("enableAtmosphericLoss",
                 "Enable external fading trace based atmospheric loss",
                 enableAtmosphericLoss);
    cmd.AddValue("metricsInterval", "Interval for link metrics logging", metricsInterval);
    cmd.Parse(argc, argv);

    // Sử dụng thư mục hiện tại (tại thời điểm chạy lệnh thường là ns-3 root)
    char cwd[1024];
    std::string pwd = (getcwd(cwd, sizeof(cwd)) != NULL) ? std::string(cwd) : ".";
    // Thêm "/../exosphere-vnu-leo" để link động vào folder dự án
    std::string projectPath = pwd + "/../exosphere-vnu-leo";

    Config::SetDefault("ns3::SatEnvVariables::DataPath",
                       StringValue(projectPath + "/data"));
    Singleton<SatEnvVariables>::Get()->SetOutputPath(projectPath + "/output");

    /// Set how satellite handle packet from GW/UT (corresponding to layers 1, 2, and 3 network)
    Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));
    Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));

    Config::SetDefault("ns3::SatOrbiterFeederPhy::QueueSize", UintegerValue(100));

    Config::SetDefault("ns3::SatHelper::HandoversEnabled", BooleanValue(true));
    Config::SetDefault("ns3::SatHandoverModule::NumberClosestSats", UintegerValue(2));
    Config::SetDefault("ns3::SatSpotBeamPositionAllocator::MinElevationAngleInDegForUT",
                       DoubleValue(10.0));
    Config::SetDefault("ns3::SatAntennaGainPattern::MinAcceptableAntennaGainDb",
                       DoubleValue(48.0));

    Config::SetDefault("ns3::SatGwMac::DisableSchedulingIfNoDeviceConnected", BooleanValue(true));
    Config::SetDefault("ns3::SatOrbiterMac::DisableSchedulingIfNoDeviceConnected",
                       BooleanValue(true));

    /// Set simulation output details
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite", BooleanValue(true));
    Config::SetDefault("ns3::SatTrafficHelper::EnableDefaultStatistics", BooleanValue(false));

    if (enableAtmosphericLoss)
    {
        Config::SetDefault("ns3::SatChannel::EnableExternalFadingInputTrace", BooleanValue(true));
    }

    /// Enable packet trace
    Config::SetDefault("ns3::SatHelper::PacketTraceEnabled", BooleanValue(true));
    
    // PERFORMANCE FIX: Disable position update on every request
    // SGP4 orbital calculations are expensive - update periodically instead
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionEachRequest", BooleanValue(false));
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionPeriod", TimeValue(Seconds(0.1)));
    std::cout << "[PERFORMANCE] SGP4 position update set to 0.1s period (not each request)" << std::endl;
    
    Ptr<SimulationHelper> simulationHelper = CreateObject<SimulationHelper>("example-gw-handover");
    Ptr<SimulationHelperConf> simulationConf = CreateObject<SimulationHelperConf>();
    simulationHelper->SetSimulationTime(Seconds(simulationTime));
    simulationHelper->SetGwUserCount(1);
    simulationHelper->SetUserCountPerUt(1);
    std::set<uint32_t> beamSetAll = {1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
                                     16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
                                     31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45,
                                     46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
                                     61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72};
    simulationHelper->SetBeamSet(beamSetAll);
    simulationHelper->SetUserCountPerMobileUt(simulationConf->m_utMobileUserCount);

    const std::string scenarioName = "constellation-leo-vnu";
    simulationHelper->LoadScenario(scenarioName);

    // to prevent automatic creation of UTs from file since we will add them as disconnected later
    g_utPositionsFilePath = projectPath + "/data/scenarios/constellation-leo-vnu/positions/ut_positions.txt";
    simulationHelper->EnableUtListPositionsFromInputFile(g_utPositionsFilePath, false);

    simulationHelper->CreateSatScenario(SatHelper::NONE);

    AddDisconnectedUtsFromFile(g_utPositionsFilePath,
                               simulationHelper->GetSatelliteHelper()->GetAntennaGainPatterns());

    if (enableLinkMetricsLog)
    {
        g_metricsInterval = metricsInterval;
        g_enableAtmosphericLoss = enableAtmosphericLoss;
        g_freeSpaceLoss = CreateObject<SatFreeSpaceLoss>();

        InitMetricsLog();

        const uint32_t utCount = Singleton<SatTopology>::Get()->GetNUtNodes();
        const uint32_t gwCount = Singleton<SatTopology>::Get()->GetNGwNodes();
        for (uint32_t i = 1; i <= utCount; ++i)
        {
            g_utMetrics[i] = LinkMetricsSample();
        }
        for (uint32_t i = 1; i <= gwCount; ++i)
        {
            g_gwMetrics[i] = LinkMetricsSample();
        }

        Config::Connect("/NodeList/*/DeviceList/*/SatPhy/PhyRx/RxCarrierList/*/LinkBudgetTrace",
                        MakeCallback(&LinkBudgetTraceCb)); // Satellite PHY
        Config::Connect(
            "/NodeList/*/DeviceList/*/UserPhy/*/PhyRx/RxCarrierList/*/LinkBudgetTrace",
            MakeCallback(&LinkBudgetTraceCb)); // UT PHY
        Config::Connect(
            "/NodeList/*/DeviceList/*/FeederPhy/*/PhyRx/RxCarrierList/*/LinkBudgetTrace",
            MakeCallback(&LinkBudgetTraceCb)); // GW PHY

        BuildUserIpMaps();

        Simulator::Schedule(g_metricsInterval, &LogMetrics);
    }

    Singleton<SatTopology>::Get()->PrintTopology(std::cout);

    if (enableFwdTraffic)
    {
        NodeContainer gwUsers = Singleton<SatTopology>::Get()->GetGwUserNodes();
        if (gwUsers.GetN() == 0)
        {
            NS_LOG_WARN("No GW users found; skipping forward traffic setup");
        }
        else
        {
            NodeContainer gwNode;
            gwNode.Add(gwUsers.Get(0));
            Time cbrInterval = Seconds(1);
            uint32_t packetSize = 2;
            uint64_t bps = static_cast<uint64_t>((packetSize * 8) / cbrInterval.GetSeconds());
            if (bps == 0)
            {
                bps = 1;
            }
            simulationHelper->GetTrafficHelper()->AddOnOffTraffic(
                SatTrafficHelper::FWD_LINK, // GW → UT
                SatTrafficHelper::UDP,
                DataRate(bps),
                packetSize,
                gwNode,
                Singleton<SatTopology>::Get()->GetUtUserNodes(),
                "ns3::ConstantRandomVariable[Constant=1000]",
                "ns3::ConstantRandomVariable[Constant=0]",
                Seconds(1.0), // start time
                Seconds(simulationTime), // end time
                Seconds(0));
        }
    }

    if (enableRtnTraffic)
    {
        NodeContainer gwUsers = Singleton<SatTopology>::Get()->GetGwUserNodes();
        if (gwUsers.GetN() == 0)
        {
            NS_LOG_WARN("No GW users found; skipping return traffic setup");
        }
        else
        {
            NodeContainer gwNode;
            gwNode.Add(gwUsers.Get(0));
            Time cbrInterval = Seconds(1);
            uint32_t packetSize = 2;
            uint64_t bps = static_cast<uint64_t>((packetSize * 8) / cbrInterval.GetSeconds());
            if (bps == 0)
            {
                bps = 1;
            }
            simulationHelper->GetTrafficHelper()->AddOnOffTraffic(
                SatTrafficHelper::RTN_LINK, // UT → GW
                SatTrafficHelper::UDP,
                DataRate(bps),
                packetSize,
                gwNode,
                Singleton<SatTopology>::Get()->GetUtUserNodes(),
                "ns3::ConstantRandomVariable[Constant=1000]",
                "ns3::ConstantRandomVariable[Constant=0]",
                Seconds(1.0), // start time
                Seconds(simulationTime), // end time
                Seconds(0.25));
        }
    }

    if (enableProgressReport)
    {
        Simulator::Schedule(Seconds(5.0), &ProgressReport);
    }

    if (enableFwdTraffic || enableRtnTraffic)
    {
        // Install on ALL nodes in the simulation
        // This ensures we capture flows regardless of routing path
        g_flowMonitor = g_flowHelper.InstallAll();
        std::cout << "[INFO] FlowMonitor installed on ALL nodes" << std::endl;
    }
    else
    {
        std::cout << "[INFO] FlowMonitor NOT installed (no traffic enabled)" << std::endl;
    }

    Config::SetDefault("ns3::SatPhyRxCarrierPerWindow::DetectionThreshold", DoubleValue(10.0));

    simulationHelper->RunSimulation();

    // Finalize FlowMonitor results at the end (no XML output)
    if (g_flowMonitor)
    {
        // IMPORTANT: CheckForLostPackets() finalizes flow statistics
        // Must be called before reading final flow statistics
        g_flowMonitor->CheckForLostPackets();

        // Debug: Print flow statistics to console
        Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(g_flowHelper.GetClassifier());
        if (classifier)
        {
            std::map<FlowId, FlowMonitor::FlowStats> stats = g_flowMonitor->GetFlowStats();
            std::cout << "[INFO] FlowMonitor captured " << stats.size() << " flows" << std::endl;
            
            for (const auto& flow : stats)
            {
                Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
                std::cout << "[FLOW] " << t.sourceAddress << " -> " << t.destinationAddress
                          << ": " << flow.second.rxPackets << " packets, "
                          << flow.second.rxBytes << " bytes" << std::endl;
            }
        }
        std::cout << "[INFO] FlowMonitor summary complete (XML output disabled)" << std::endl;
    }
    else
    {
        std::cout << "[INFO] No FlowMonitor to serialize (traffic was disabled)" << std::endl;
    }

    return 0;
}