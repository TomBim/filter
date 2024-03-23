echo "****  LES GO! LES BUILD IT!  *****"

cd build
rm CMakeCache.txt
cmake -GNinja .. -DKALMAN=1 -DFILTER_V_MAJOR=2 -DFILTER_V_MINOR=0

echo ""
echo "*****   NINJA NELES!!!   *****"
ninja #> /mnt/c/Users/tombim/OneDrive/Bureau/log.md
ninja install
#cmake --install .

echo ""
echo "*****   EXECUTINGGG....  *****"
cd ..
./bin/simulador #> /mnt/c/Users/tombim/OneDrive/Bureau/log.md