#pragma once

/**
* Modified version of mourner's kdbush.hpp to support old C++
* https://github.com/mourner/kdbush.hpp
*/

#undef min
#undef max

#include <algorithm>
#include <cmath>
// Both 
//#include <cstdint>
//#include <tuple>
#include <vector>
#include <cassert>

namespace kdbush {

    template <typename TPoint>
    class KDBush {
    public:
        KDBush(const std::vector<TPoint> input, std::size_t nodeSize_ = 64) :
            nodeSize(nodeSize_)
        {

            const std::size_t size = input.size();

            points.reserve(size);
            ids.reserve(size);

            for (unsigned int i = 0; i < input.size(); i++)
            {
                // TODO:  Use xy or something here...
                //points.push_back(std::pair<int,int>(input[i].first, input[i].second));
                points.push_back(std::pair<int, int>(input[i].first, input[i].second));
                ids.push_back(i);
            }
            sortKD(0, size - 1, 0);
        }

        void range(const int minX,
            const int minY,
            const int maxX,
            const int maxY,
            std::vector<std::size_t> &out) {
            range(minX, minY, maxX, maxY, out, 0, static_cast<std::size_t>(ids.size() - 1), 0);
        }


    private:


        void range(const int minX,
            const int minY,
            const int maxX,
            const int maxY,
            std::vector< std::size_t >& out,
            const std::size_t left,
            const std::size_t right,
            const std::size_t axis) {

            if (right - left <= nodeSize) {
                for (std::size_t i = left; i <= right; i++) {
                    const int x = points[i].first;
                    const int y = points[i].second;
                    if (x >= minX && x <= maxX && y >= minY && y <= maxY) out.push_back(ids[i]);
                }
                return;
            }

            const std::size_t m = (left + right) >> 1;
            const int x = points[m].first;
            const int y = points[m].second;

            if (x >= minX && x <= maxX && y >= minY && y <= maxY) out.push_back(ids[m]);

            if (axis == 0 ? minX <= x : minY <= y)
                range(minX, minY, maxX, maxY, out, left, m - 1, (axis + 1) % 2);

            if (axis == 0 ? maxX >= x : maxY >= y)
                range(minX, minY, maxX, maxY, out, m + 1, right, (axis + 1) % 2);
        }

        int get(const std::pair< int, int >& p, unsigned int axis) {
            return axis == 0 ? p.first : p.second;
        }

        void sortKD(const std::size_t left, const std::size_t right, const unsigned int axis) {
            if (right - left <= nodeSize) return;
            const std::size_t m = (left + right) >> 1;
            select(m, left, right, axis);
            sortKD(left, m - 1, (axis + 1) % 2);
            sortKD(m + 1, right, (axis + 1) % 2);
        }


        void select(const std::size_t k, std::size_t left, std::size_t right, const unsigned int axis) {

            while (right > left) {
                if (right - left > 600) {
                    const double n = right - left + 1;
                    const double m = k - left + 1;
                    const double z = std::log(n);
                    const double s = 0.5 * std::exp(2 * z / 3);
                    const double r =
                        k - m * s / n + 0.5 * std::sqrt(z * s * (1 - s / n)) * (2 * m < n ? -1 : 1);
                    select(k, std::max(left, std::size_t(r)), std::min(right, std::size_t(r + s)), axis);
                }

                const int t = get(points[k], axis);
                std::size_t i = left;
                std::size_t j = right;

                swapItem(left, k);
                if (get(points[right], axis) > t) swapItem(left, right);

                while (i < j) {
                    swapItem(i++, j--);
                    while (get(points[i], axis) < t) i++;
                    while (get(points[j], axis) > t) j--;
                }

                if (get(points[left], axis) == t)
                    swapItem(left, j);
                else {
                    swapItem(++j, right);
                }

                if (j <= k) left = j + 1;
                if (k <= j) right = j - 1;
            }
        }

        void swapItem(const std::size_t i, const std::size_t j) {
            std::iter_swap(ids.begin() + i, ids.begin() + j);
            std::iter_swap(points.begin() + i, points.begin() + j);
        }

        int sqDist(const int ax, const int ay, const int bx, const int by) {
            return std::pow(ax - bx, 2) + std::pow(ay - by, 2);
        }

        std::vector<std::size_t> ids;
        std::vector< std::pair< int, int > > points;
        std::size_t nodeSize;
    };
}
