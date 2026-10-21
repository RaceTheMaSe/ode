#! /bin/bash
if ! command -v codespell &> /dev/null
then
    echo "codespell not found - skipping"
    exit 0
fi
declare -A excludeList
        excludeList[".github"]=""
        excludeList["bindings"]="-L stdio"
        excludeList["build"]=""
        excludeList["cmake"]="-L SEH"
        excludeList["drawstuff"]="-L bu,bult,cyle,UINT,usre"
        excludeList["GIMPACT"]="-L endcode"
        excludeList["include"]="-L endcode,lets,master,stdio"
        excludeList["libccd"]="-L stdio"
        excludeList["ode"]="-L alo,amin,ang,clen,DOF,fo,lets,numer,OD,ro,sinc,tempRay,thi"
        excludeList["OPCODE"]="-L endcode,indx,ith,NMAKE,Od,stdio,Tim"
        excludeList["ou"]="-L endcode,NMAKE,OBJEXT,Od,stdio"
        excludeList["tests"]="-L lets"
allResult=0
for key in ${!excludeList[@]}; do
    cd ../$key
    codespell --builtin 'clear,rare,informal,usage,code,names' ${excludeList[${key}]}
    result=$?
    if [ $result -ne 0 ] ; then
        allResult=1
    fi
done
if [ $allResult -ne 0 ] ; then
    exit 1
fi
