#!/usr/bin/env bash

VSCode=$1
gitKey=$2


totalFiles=0
downloaded=0
downfiles=0

spin() {
	Percent=$((100 * downloaded / totalFiles))
	BarLen=$((Percent / 5))
	printf "\r["
	
	for j in {1..20}
	do
		if [ "$j" -gt "$BarLen" ]
		then printf "%s" " "
		else printf "%s" "="
		fi
	done

	printf "] %d%%" "$Percent" 
}
endspin() {
   printf "\r["
	
	for j in {1..20}
	do
		printf "%s" "="
	done

	printf "] 100%%\r\n"
}

downloadGitRec() {
	local tmp1=$1
	local tmp2=$2
	
	if [ $downfiles -eq 0 ]
	then
		mkdir ${tmp1}
	fi

	cd ${tmp1}

	if [ $downfiles -eq 0 ]
	then
		curl -H 'Authorization: token '${gitKey}'' -H 'Accept: application/vnd.github.v3.raw' -o 'bashtmp' -L https://api.github.com/repos/Lapland-Robotics/RescueDrone/contents/${tmp2} &> /dev/null
	fi
	
	readarray -t arraytmp < <(jq -r .[].type bashtmp)
	local type=(${arraytmp[@]})
	readarray -t arraytmp < <(jq -r .[].path bashtmp)
	local location=(${arraytmp[@]})
	readarray -t arraytmp < <(jq -r .[].name bashtmp)
	local name=(${arraytmp[@]})

	local len=${#type[@]}
	
	local i=0
	while [ $i -lt $len ]
	do
		if [ $downfiles -eq 1 ]; then spin; fi

		if [ "${type[$i]}" = "dir" ]
		then
			downloadGitRec ${name[$i]} ${location[$i]}

		else
			if [ $downfiles -eq 0 ]
			then
				let "totalFiles++"
			else
				curl -H 'Authorization: token '${gitKey}'' -H 'Accept: application/vnd.github.v3.raw' -O -L https://api.github.com/repos/Lapland-Robotics/RescueDrone/contents/${location[$i]} &> /dev/null
				let "downloaded++"
			fi
		fi
		let "i++"
	done

	if [ $downfiles -eq 1 ]
	then
		rm bashtmp
	fi
	cd ..
}

downloadSRCFiles(){
	downloadGitRec $1 $1
	echo "start downloading source files"
	downfiles=1
	downloadGitRec $1 $1
	endspin
	echo "finished downloading the files"
}

sudo apt-get update 
sudo apt-get autoremove -y 
sudo apt-get upgrade -y 
sudo apt-get install gcc cmake curl nano -y 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 
sudo apt update 
sudo apt install ros-melodic-desktop-full -y 
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
echo "export EDITOR='nano -w'" >> ~/.bashrc 
source /opt/ros/melodic/setup.bash 
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y 
sudo rosdep init 
rosdep update 
sudo apt install ros-melodic-nmea-comms -y 
sudo apt-get install libusb-1.0-0-dev -y 
cd ~/ 
mkdir catkin_ws  cd catkin_ws 
mkdir src  cd src 
git clone -b 3.8 https://github.com/dji-sdk/Onboard-SDK-ROS.git Onboard-SDK 
curl https://pastebin.com/raw/bUZwn3EY > Onboard-SDK/dji_sdk/src/modules/dji_sdk_node.cpp 
rosdep install -y --rosdistro melodic --from-paths src --ignore-src 
downloadSRCFiles Onboard-computer
cd .. 
catkin_make_isolated 
echo "source devel_isolated/setup.bash" >> ~/.bashrc 
source ~/catkin_ws/devel_isolated/setup.bash 
sudo adduser $USER dialout 

if [ $VSCode -eq 1 ]
then
	cd ~/ 
	git clone https://github.com/jetsonHacksNano/installVSCode.git 
	cd installVSCode 
	sudo ./installVSCode.sh 
fi

sudo reboot