#ifndef SPD_TYPES_H
#define SPD_TYPES_H

#include <vector>

enum class SpdType : int{ RobotTrue, RobotEstimation, WheelsTrue, WheelsRead, NTypes };

// template function for vectors with a value for each speed type
template <typename T>
class vecEachSpdType{
    public:
        std::vector<T> vec;

        vecEachSpdType() : vec(static_cast<int>(SpdType::NTypes)) {}

        vecEachSpdType(const std::vector<SpdType> &types, const std::vector<T> &values) : vec(static_cast<int>(SpdType::NTypes)) {
            int valuesSize = values.size();
            if(types.size() == valuesSize && valuesSize == static_cast<int>(SpdType::NTypes))
                for(int i = 0; i < valuesSize; i++)
                    vec[static_cast<int>(types[i])] = values[i];
        }

        T at(SpdType type) const { return vec.at(static_cast<int>(type)); }

        T at(int index) const { return vec.at(index); }

        void putAt(SpdType type, T input) { vec[static_cast<int>(type)] = input; }

        void putAt(int index, T input) { vec[index] = input; }
};

#endif