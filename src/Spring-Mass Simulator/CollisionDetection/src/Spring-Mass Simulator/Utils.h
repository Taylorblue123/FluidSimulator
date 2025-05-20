#pragma once
#include <cstdlib> // For rand() and srand()
#include <iostream> // For std::cout and std::endl
#include <vector>
#include <unordered_set>
#include <functional>
#include <algorithm>
#include <ctime>

class Utils
{
public:
    Utils() {
        // Initialize random seed
        srand(static_cast<unsigned int>(time(0)));
    }

    float randomFloat(float min, float max) {
        return min + (max - min) * (rand() / (float)RAND_MAX);
    }

    int randomInt(int min, int max) {
        return min + (rand() % (max - min + 1));
    }

    // A generic insertion sort that sorts the vector in ascending order.
    template<typename T>
    void insertionSort(std::vector<T>& arr) {
        int n = arr.size();
        for (int i = 1; i < n; i++) {
            T key = arr[i];
            int j = i - 1;
            while (j >= 0 && (key < arr[j])) {
                arr[j + 1] = arr[j];
                j--;
            }
            arr[j + 1] = key;
        }
    }

    // Updated threeSetIntersectionUnordered function with better efficiency
    template<typename T>
    std::vector<T> threeSetIntersectionUnordered(const std::vector<T>& A,
                                                  const std::vector<T>& B,
                                                  const std::vector<T>& C) {
        if (A.empty() || B.empty() || C.empty()) {
            return std::vector<T>();
        }

        const std::vector<T>* smallest = &A;
        const std::vector<T>* other1 = &B;
        const std::vector<T>* other2 = &C;

        if (B.size() < smallest->size()) {
            smallest = &B;
            other1 = &A;
            other2 = &C;
        }
        if (C.size() < smallest->size()) {
            smallest = &C;
            other1 = &A;
            other2 = &B;
        }

        std::unordered_set<T> setOther1(other1->begin(), other1->end());
        std::unordered_set<T> setOther2(other2->begin(), other2->end());

        std::vector<T> result;
        for (const T& x : *smallest) {
            if (setOther1.count(x) && setOther2.count(x)) {
                result.push_back(x);
            }
        }

        return result;
    }
};