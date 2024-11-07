#!usr/bin/bash

if [ -n "$1" ] ; then
    while true; do
        case "$1" in
        -u | --use_rviz)
            use_rviz=1
            break
            ;;
        -h | --help)
            echo -e "Usage:\n bash $0 [options]\n"
            echo -e "Options:\n --use_rviz, -u : 使用Rviz显示\n --help,     -h : 帮助\n"
            echo -e "Others:\n 正常运行需要使用net-tools，使用以下命令安装：sudo apt install net-tools 。"
            exit 0
            ;;
        *)
            echo "Wrong option, pls check!"
            exit 1
            ;;
        esac
    done
else
    use_rviz=0
fi

if [ -f build/devel/setup.bash ];then
    :
else
    if [ -f build.sh];then
        bash build.sh
    else
        :
    fi
fi

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

if [ -d launch/ ];then
    if [ -f launch/sensing.launch ];then
        source build/devel/setup.bash
        roslaunch sensing sensing.launch use_rviz:=$use_rviz
    else
        echo "[Shell] <------ The launch file doesn't exist. ------>"
    fi
else
    if [ -f build/sensing];then
        gnome-terminal -t "roscore" -- bash -c "roscore;exec bash;"
        sleep 1s
        ./build/devel/lib/sensing/sensing
    else
        echo "[Shell] <------------- No executables --------------->"
    fi
fi


