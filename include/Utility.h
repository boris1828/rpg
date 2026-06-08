#pragma once

#include <stdlib.h>
#include <optional>
#include <iostream>

#include "Macro.h"

template<typename T, std::size_t N>
struct SmallSet
{
    T data[N];
    std::size_t count = 0;

    void clear() { count = 0; }

    std::size_t size() const { return count; }

    void add(const T& t) 
    {
        if (contains(t)) return;

        ASSERT(count < N, "SmallSet capacity exceeded! Consider increasing N.");
        
        if (count < N) 
        {
            data[count++] = t;
        }
    }

    void add(const SmallSet<T, N>& set) 
    {
        for (const auto& t : set)
        {
            add(t);
        }
    }

    void remove(const T& t) 
    {
        for (std::size_t i = 0; i < count; ++i) 
        {
            if (data[i] == t) 
            {
                data[i] = data[count - 1];
                count--;
                return;
            }
        }
    }

    bool contains(const T& t) const 
    {
        for (std::size_t i = 0; i < count; ++i) 
        {
            if (data[i] == t) return true;
        }
        return false;
    }

    bool empty() const { return count == 0; };

    T* begin() { return &data[0]; }
    T* end()   { return &data[count]; }
    const T* begin() const { return &data[0]; }
    const T* end()   const { return &data[count]; }
};

template<typename T, std::size_t N>
SmallSet<T, N> Difference(const SmallSet<T, N>& s1, const SmallSet<T, N>& s2)
{
    SmallSet<T, N> result;

    for (const auto& item : s1)
        if (!s2.contains(item))
            result.add(item);

    return result;
}

template<typename T, std::size_t N>
SmallSet<T, N> SymmetricDifference(const SmallSet<T, N>& s1, const SmallSet<T, N>& s2)
{
    SmallSet<T, N> result;

    for (const auto& item : s1)
        if (!s2.contains(item))
            result.add(item);

    for (const auto& item : s2)
        if (!s1.contains(item))
            result.add(item);

    return result;
}

template<typename T, std::size_t N>
SmallSet<T, N> Union(const SmallSet<T, N>& s1, const SmallSet<T, N>& s2)
{
    SmallSet<T, N> result = s1;

    for (const auto& item : s2)
        result.add(item); 

    return result;
}

template<typename T, std::size_t N>
SmallSet<T, N> Intersection(const SmallSet<T, N>& s1, const SmallSet<T, N>& s2)
{
    SmallSet<T, N> result;

    for (const auto& item : s1)
        if (s2.contains(item))
            result.add(item);

    return result;
}


// Small Map

template<typename K, typename V, size_t MaxSize>
struct StaticMap 
{
    K keys[MaxSize];
    V values[MaxSize];
    size_t currentSize = 0;

    bool set(K key, V value) 
    {
        ASSERT(!hasKey(key), "should not reset value of key");

        if (currentSize < MaxSize) 
        {
            keys[currentSize]   = key;
            values[currentSize] = value;
            currentSize++;
            return true;
        }

        WARNING(true, "Small map is full");
        return false;
    }

    V get(K key) const 
    {
        for (size_t i = 0; i < currentSize; ++i) 
        {
            if (keys[i] == key) 
            {
                return values[i];
            }
        }

        ASSERT(false, "if called get should posses key");
    }

    std::optional<V> tryGet(K key) const
    {
        for (size_t i = 0; i < currentSize; ++i) 
            if (keys[i] == key) 
                return values[i]; 

        return std::nullopt; 
    }

    bool hasKey(K key) const 
    {
        for (size_t i = 0; i < currentSize; ++i) 
            if (keys[i] == key) return true;

        return false;
    }
};