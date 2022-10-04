PASSWORD=HappyLab.2021
echo $PASSWORD | sudo -S chmod a+rw /dev/ttyACM_IRMAGICIAN
echo $PASSWORD | sudo -S chmod a+rw /dev/ttyURG0
echo $PASSWORD | sudo -S chmod a+rw /dev/ttyURG1
#echo $PASSWORD | sudo -S chmod a+rw /dev/ttyARDUINO
echo $PASSWORD | sudo -S ip link set can0 up type can bitrate 500000
echo "success"
