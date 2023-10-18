echo "les go! les build it"

cd build
#rm CMakeCache.txt
cmake -GNinja ..
ninja
ninja install
#cmake --install .


echo "now, les execute it shauhasusha"
cd ..
./bin/simulador > /mnt/c/Users/tombim/OneDrive/Bureau/log.md