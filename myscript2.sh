filterDirectives=(
    "-DNO_FILTER=1"
    "-DKALMAN=1 -DFILTER_V_MAJOR=1 -DFILTER_V_MINOR=0"
    "-DKALMAN=1 -DFILTER_V_MAJOR=2 -DFILTER_V_MINOR=0"
    "-DINFO=1 -DFILTER_V_MAJOR=1 -DFILTER_V_MINOR=0"
    "-DINFO=1 -DFILTER_V_MAJOR=2 -DFILTER_V_MINOR=0"
    )

for filterDirective in "${filterDirectives[@]}"; do
    echo ""
    echo "▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓"
    echo ""

    echo "****  LES GO! LES BUILD IT!  *****"

    cd build
    rm * -r
    cmake -GNinja --log-level=WARNING .. ${filterDirective}

    echo ""
    echo "*****   NINJA NELES!!!   *****"
    ninja #> /mnt/c/Users/tombim/OneDrive/Bureau/log.md
    ninja install
    #cmake --install .

    echo ""
    echo "*****   EXECUTINGGG....  *****"
    cd ..
    ./bin/simulador #> /mnt/c/Users/tombim/OneDrive/Bureau/log.md
done