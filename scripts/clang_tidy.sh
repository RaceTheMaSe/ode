#! /bin/bash
# from: https://unix.stackexchange.com/a/446200
# interruptible - combination of https://stackoverflow.com/a/14702379 and https://www.baeldung.com/linux/just-started-process-pid
clangTidyBin=clang-tidy
if ! command -v clang-tidy &> /dev/null
then
    echo "clang-tidy not found in ${clangTidyBin}"
    exit 0
fi
clangVersion=$(${clangTidyBin} --version | grep -Po '( )([0-9.]+)' | sed -E 's/ ([0-9]+){1}.(.*)/\1/g;t;d') # fragile!
echo "clang-tidy ${clangVersion} check - this may take a long time"
# array of files, assuming at least 1 item exists
files=($(find ../ode ../include -name \*.cpp -o -name \*.h)) # header files only work without precompiled headers, remove them otherwise
# files=($(cat filelist.txt)) # that was created with above find command and filtered

currDir=$(dirname -- $(readlink -fn -- "$0"))
buildDir="${currDir}/../build-clang"
cd ${buildDir}
abort=false
pidWorkers=()
exitfn () {
  trap SIGINT            # Restore signal handling for SIGINT.
  for pid in "${pidWorkers[@]}"
  do
    kill $pid
  done
  echo; echo 'Aborted'
  exit
}
trap "exitfn" INT        # Set SIGINT trap to call function.

# task
task() { # $1 = idWorker, $2 = asset
  echo "Worker $1: $2"
  echo "$2:" >> ../scripts/clang_tidy_err$1.log
  cmd_invokation="${clangTidyBin} -p ${buildDir}/compile_commands.json --config-file=${currDir}/../.clang-tidy $2"
  ${cmd_invokation} >> ../scripts/clang_tidy_log$1.log 2>> ../scripts/clang_tidy_err$1.log
  # echo "${cmd_invokation}"
  result=$?
  if [ $result -ne 0 ] ; then
      printf "Error with $1: $2\n"
  fi
}

nVirtualCores=$(nproc --all)
nWorkers=$(( $nVirtualCores * 1 )) # 1 process per core

worker() { # $1 = idWorker
  echo "Worker $1 GO!"
  idAsset=0
  for asset in "${files[@]}"; do
    # split assets among workers (using modulo); each worker will go through
    # the list and select the asset only if it belongs to that worker
    if [ $abort = false ] ; then
      (( idAsset % nWorkers == $1)) && task $1 "$asset"
    fi
    (( idAsset++ ))
  done
  echo "    Worker $1 ALL DONE!"
}

for (( idWorker=0; idWorker<nWorkers; idWorker++ )); do
  # start workers in parallel, use 1 process for each
  worker $idWorker &
  pidWorkers+="$! "
done
wait # until all workers are done

cd ${currDir}
echo ''
echo "Done"
