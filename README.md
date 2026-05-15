How to run?

Sau khi cài đặt sns3, nó sẽ là thư mục ns-3/contrib

Ở cùng nơi để thư mục ns-3, clone repo này

Copy NỘI DUNG thư mục ns-3/contrib/satellite/data/scenarios/constellation-leo-2-satellites vào thư mục exosphere-vnu-leo/data/scenarios/constellation-leo-vnu

Copy cả ns-3/contrib/satellite/data/additional-input vào exosphere-vnu-leo/data/additional-input

Chạy 2 lệnh sau: 

ln -s /home/osboxes/VNU-LEO/exosphere-vnu-leo/data/scenarios/constellation-leo-vnu /home/osboxes/VNU-LEO/ns-3/contrib/satellite/data/scenarios/constellation-leo-vnu

ln -s /home/osboxes/VNU-LEO/exosphere-vnu-leo/vnu-leo-final.cc /home/osboxes/VNU-LEO/ns-3/scratch/vnu-leo-final.cc

Ở thư mục exosphere-vnu-leo/, chạy command dưới để bắt đầu simulation:

../ns-3/ns3 run vnu-leo-final -- --simTime=100 2>&1 | tee handover.log
