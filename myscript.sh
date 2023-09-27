echo "les go! les build it"

cd build
rm CMakeCache.txt
cmake -GNinja ..
ninja
ninja install

echo "now, les execute it shauhasusha"
./simulador
