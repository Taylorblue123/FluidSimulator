#pragma once
#include <cstdlib> // For rand() and srand()
#include <iostream> // For std::cout and std::endl
#include <vector>
#include <unordered_set>
#include <functional>
#include "SphereBV.h"

class Utils
{
    public:
        float randomFloat(float min, float max){
            return min + (max - min) * (rand() / (float) RAND_MAX);
        }

        int randomInt(int min, int max){ // Changed return type to int
            return min + (rand() % (max - min + 1));
        }
        
        // A generic insertion sort that sorts the vector in ascending order.
        template<typename T>
        void insertionSort(std::vector<T>& arr) {
            int n = static_cast<int>(arr.size());
            for (int i = 1; i < n; i++) {
                T key = arr[i];
                int j = i - 1;
                // Here we use the operator< to compare key and arr[j].
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
            // If any set is empty, the intersection is empty
            if (A.empty() || B.empty() || C.empty()) {
                return std::vector<T>();
            }
            
            // Find the smallest set to iterate through
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
            
            // Create hash sets for O(1) lookup in other sets
            std::unordered_set<T> setOther1;
            setOther1.reserve(other1->size()); // Reserve space for better performance
            for (const T& elem : *other1) {
                setOther1.insert(elem);
            }
            
            std::unordered_set<T> setOther2;
            setOther2.reserve(other2->size()); // Reserve space for better performance
            for (const T& elem : *other2) {
                setOther2.insert(elem);
            }
            
            // Find elements that exist in all three sets
            std::vector<T> result;
            result.reserve(smallest->size()); // Pre-allocate for efficiency
            for (const T& x : *smallest) {
                if (setOther1.count(x) && setOther2.count(x)) {
                    result.push_back(x);
                }
            }
            
            return result;
        }
};

// Add hash specialization for std::pair<SphereBV, SphereBV>
namespace std {
    template <>
    struct hash<std::pair<SphereBV*, SphereBV*>> {
        size_t operator()(const std::pair<SphereBV*, SphereBV*>& p) const {
            size_t h1 = hash<int>()(p.first->id);
            size_t h2 = hash<int>()(p.second->id);
            // Combine hashes (boost::hash_combine implementation)
            return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
        }
    };
}