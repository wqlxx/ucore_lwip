echo " [ MODULE ] loading vbox kernel module ..."
for i in vboxdrv vboxnetflt vboxnetadp; do
    echo "            $i"
    modprobe $i || (echo "modprobe $i failed, please run '/etc/rc.d/vboxdrv setup' first" && exit)
done

VBoxManage list vms | grep ucore

if [[ $? == 0 ]]; then
    echo " [ DELETE ] existing ucore"
    VBoxManage unregistervm ucore --delete || (echo "posibly ucore is running, please shut it down first" && exit)
fi

echo " [ CONVERT ] img to vdi"
for i in bin/*.img; do
    if [ ! -f ${i%%.*}.vdi ];then
        echo $i
        VBoxManage convertfromraw $i ${i%%.*}.vdi
    fi
done

echo " [ CREATE ] ucore vm"
VBoxManage createvm --name ucore --register || exit

echo " [ CONFIG ] ucore "
VBoxManage modifyvm ucore --memory 500 --cpus 1

echo " [ CONFIG ] add IDE controller"
VBoxManage storagectl ucore --name "IDE Controller" --add ide --controller PIIX4 --bootable on

echo " [ ATTACH ] attach vdi to IDE controller"

echo "            adding bin/ucore.vdi"
if [ -f bin/ucore.vdi ];then
    VBoxManage storageattach ucore --storagectl "IDE Controller" --port 0 --device 0 --type hdd --medium bin/ucore.vdi
else
    echo " [ ERROR ] bin/ucore.vdi missing ..."
    exit
fi

echo "            adding bin/swap.vdi"
if [ -f bin/swap.vdi ];then
    VBoxManage storageattach ucore --storagectl "IDE Controller" --port 0 --device 1 --type hdd --medium bin/swap.vdi
else
    echo " [ ERROR ] bin/swap.vdi missing"
    exit
fi

echo "            adding bin/fs.vdi"
if [ -f bin/fs.vdi ];then
    VBoxManage storageattach ucore --storagectl "IDE Controller" --port 1 --device 0 --type hdd --medium bin/fs.vdi
else
    echo " [ ERROR ] bin/fs.vdi missing"
    exit
fi

echo " [ NETWORK ] adding bridge adapter"
VBoxManage modifyvm ucore --nic1 bridged --nictype1 82540EM --cableconnected1 on --bridgeadapter1 eth0

echo " [ SERIAL ] adding serial port"
VBoxManage modifyvm ucore --uart1 0x3F8 4 --uartmode1 server "/tmp/vb.serial"
