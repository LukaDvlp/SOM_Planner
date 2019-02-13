#!/bin/bash
#Environment1
ExpNum=1
TryNum=1
Name="Test"${ExpNum}"_Result"
Dir="./"${Name}
csv=${Name}"/"${Name}".csv"
if [ -e ${Dir} ]; then
	rm -r ${Dir}
fi
mkdir ${Dir}
:>| ${csv}

GNum=121
#GPosX=10
#GPosY=10
AddGoalNum=2
MaxIte=1500

P1_g=1
P2_g=0
P3_g=2
P4_g=1
P1_s=0 #G0
P2_s=1 #Mu
P3_s=0 #Alpha
P4_s=2 #NI
#Loop1=(0 1 2 3 4 5 6 7 8 9) #Environment Number (New goals distribution)
#Loop2=(0 1 2) #Trial Number 
Loop1=(0) 
Loop2=(0)

#python3 ./InitCSV.py ${csv}
python3 ./CreateWorld_AA.py ${GNum} 

#python3 ./Init_Solution_GA.py ${MaxIte} ${P1_g} ${P2_g} ${P3_g} ${P4_g} ${Dir} ${GPosX} ${GPosY}
#python3 ./Init_Solution_SOM.py ${GNum} ${MaxIte} ${P1_s} ${P2_s} ${P3_s} ${P4_s} ${Dir} ${GPosX} ${GPosY}
python3 ./Init_Solution_SOM.py ${GNum} ${MaxIte} ${P1_s} ${P2_s} ${P3_s} ${P4_s} ${Dir}

