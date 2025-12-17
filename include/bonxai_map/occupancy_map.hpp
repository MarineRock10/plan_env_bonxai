#pragma once

#include <eigen3/Eigen/Geometry>  // Eigen几何库，用于3D向量运算
#include <unordered_set>           // 无序集合，用于高效查找
#include <cmath>                  // 数学函数
#include <algorithm>              // 算法函数

#include "bonxai_core/bonxai.hpp"  // Bonxai核心库，包含VoxelGrid等基础数据结构

namespace Bonxai
{
    namespace MapUtils
    {
        /**
         * @brief 计算对数概率（log-odds），结果以整数形式返回
         * @details 将概率值转换为对数概率，用于概率更新计算
         *          公式: log(p / (1-p)) * 1e6
         *          乘以1e6是为了将浮点数转换为整数，避免浮点运算开销
         * @param float probability 原始概率值，范围(0,1)
         * @return int32_t 对数概率的整数表示
         */
        [[nodiscard]] inline int32_t logods(float prob)
        {
            return int32_t(1e6 * std::log(prob / (1.0 - prob)));
        }

        /**
         * @brief 从对数概率计算原始概率
         * @details 公式: 1 - 1/(1 + exp(logodds))
         *          这是logods函数的逆运算
         * @param int32_t logods_fixed 对数概率的整数表示
         * @return float 原始概率值
         */
        [[nodiscard]] inline float prob(int32_t logods_fixed)
        {
            float logods = float(logods_fixed) * 1e-6;  // 从定点数转为浮点数
            return (1.0 - 1.0 / (1.0 + std::exp(logods)));  // 将log-odds转回概率
        }

        /**
         * @brief 射线迭代器，对射线上的每个体素执行操作
         * @details 使用数字微分分析（DDA）算法遍历射线上的体素
         *          算法特点：整数运算，无浮点误差，高效遍历
         * @tparam Functor 函数对象类型，接受CoordT参数，返回bool表示是否继续
         * @param CoordT key_origin 射线起点坐标（体素坐标）
         * @param CoordT key_end 射线终点坐标（体素坐标）
         * @param Functor func 函数对象，对每个经过的体素执行操作
         *                     func返回false时提前终止迭代
         */
        template <class Functor>
        inline void RayIterator(const CoordT& key_origin, const CoordT& key_end, const Functor& func)
        {
            // 如果起点和终点相同，不需要遍历
            if (key_origin == key_end) {return;}
            
            // 对起点执行操作，如果返回false则直接返回
            if (!func(key_origin)) {return;}

            CoordT error = {0, 0, 0};  // 误差累积，用于决定何时在某个维度上移动
            CoordT coord = key_origin;  // 当前遍历的坐标
            CoordT delta = (key_end - coord);  // 终点与起点的差值
            const CoordT step = {delta.x < 0 ? -1 : 1,  // x方向步长
                                delta.y < 0 ? -1 : 1,  // y方向步长
                                delta.z < 0 ? -1 : 1}; // z方向步长

            // 计算绝对值，用于误差判断
            delta = {
                delta.x < 0 ? -delta.x : delta.x, 
                delta.y < 0 ? -delta.y : delta.y,
                delta.z < 0 ? -delta.z : delta.z};

            const int max = std::max(std::max(delta.x, delta.y), delta.z);  // 最大维度差值

            // 最大变化次数决定迭代次数
            for (int i = 0; i < max - 1; ++i) 
            {
                // 更新误差累积
                error = error + delta;
                
                // 手动循环展开，分别处理x、y、z三个维度
                // 当误差累积达到阈值时，在该维度上前进一步
                if ((error.x << 1) >= max)  // 等价于 2*error.x >= max
                {
                    coord.x += step.x;  // 在x方向移动一步
                    error.x -= max;     // 减去最大值，避免误差过大
                }

                if ((error.y << 1) >= max) 
                {
                    coord.y += step.y;
                    error.y -= max;
                }

                if ((error.z << 1) >= max) 
                {
                    coord.z += step.z;
                    error.z -= max;
                }

                // 对当前坐标执行操作
                if (!func(coord)) {return;}
            }
        }

        /**
         * @brief 计算两点之间的射线路径
         * @details 生成从起点到终点（不包括终点）的所有体素坐标
         * @param CoordT key_origin 射线起点
         * @param CoordT key_end 射线终点
         * @param std::vector<CoordT>& ray 用于存储射线路径的输出向量
         */
        inline void ComputeRay(const CoordT& key_origin, const CoordT& key_end, std::vector<CoordT>& ray)
        {
            // 清空射线向量
            ray.clear();
            
            // 创建用于收集坐标的函数对象
            auto functor = [&ray](const CoordT& coord)
            {
                ray.push_back(coord);  // 将每个坐标添加到射线向量
                return true;           // 继续遍历
            };

            // 使用射线迭代器计算射线路径
            RayIterator(key_origin, key_end, functor);
        }

        /**
         * @brief 占用网格的参数配置结构体
         * @details 使用对数概率表示，避免浮点运算
         * @property int32_t prob_miss_log 未命中的对数概率增量（负数）
         * @property int32_t prob_hit_log 命中的对数概率增量（正数）
         * @property int32_t clamp_min_log 概率下限的对数表示
         * @property int32_t clamp_max_log 概率上限的对数表示
         * @property int32_t occupancy_threshold_log 占用阈值的对数表示
         *          概率高于此阈值认为被占用
         */
        struct OccupancyOptions
        {
            int32_t prob_miss_log = logods(0.4f);      // 未命中概率0.4对应的对数概率
            int32_t prob_hit_log = logods(0.7f);       // 命中概率0.7对应的对数概率
            
            int32_t clamp_min_log = logods(0.12f);     // 最小概率限制0.12
            int32_t clamp_max_log = logods(0.97f);    // 最大概率限制0.97
            
            int32_t occupancy_threshold_log = logods(0.5);  // 占用阈值0.5

            double resolution = 0.05;
            double inflation_radius = 0.1;
        };

        // 必须在 CellOcc 之前定义 UnknownProbability
        const int32_t UnknownProbability = logods(0.5f);  // 未知概率的对数表示

        /**
         * @brief 占用网格的单个体素数据结构
         * @property int32_t update_id 更新ID，用于标识当前更新周期
         *          值在1-3之间循环，避免同一周期重复更新
         * @property int32_t probability_log 占用概率的对数表示
         *          初始值为28，对应概率约0.5
         */
        struct CellOcc
        {
            int32_t update_id {4};          // 更新ID，4表示未初始化
            int32_t probability_log {28};   // 对数概率，28 = logods(0.5)*1e6

            CellOcc()  // 默认构造函数
                : update_id(4)
                , probability_log(UnknownProbability)  // 初始化为未知状态（概率0.5）
            {}
        };
    }

    /**
     * @brief 3D占用网格地图类
     * @details 基于对数概率的占用网格表示，支持动态更新和查询
     *          使用Bonxai的稀疏体素网格作为底层数据结构
     */
    class OccupancyMap
    {
    public:
        // 使用Eigen向量类型别名
        using Vector3D = Eigen::Vector3d;

        /**
         * @brief 构造函数，仅接受分辨率参数
         * @details 用户需要后续手动设置options
         * @param double resolution 体素分辨率（米）
         */
        explicit OccupancyMap(double resolution)
            : grid_(resolution)              // 创建体素网格
            , accessor_(grid_.createAccessor())  // 创建网格访问器
            , accessor_bound_(true)          // 标记访问器已绑定
        {}

        /**
         * @brief 构造函数，接受分辨率和选项参数
         * @details 推荐使用此构造函数创建新地图
         * @param double resolution 体素分辨率
         * @param MapUtils::OccupancyOptions& options 占用网格选项
         */
        explicit OccupancyMap(double resolution, MapUtils::OccupancyOptions& options)
            : grid_(resolution)              // 创建体素网格
            , options_(options)              // 设置占用选项
            , accessor_(grid_.createAccessor())  // 创建网格访问器
            , accessor_bound_(true)          // 标记访问器已绑定
        {}

        /**
         * @brief 构造函数，接受选项和已存在的网格
         * @details 用于从反序列化的网格创建地图
         * @param MapUtils::OccupancyOptions& options 占用选项
         * @param VoxelGrid<MapUtils::CellOcc>&& grid 移动的体素网格
         */
        explicit OccupancyMap(MapUtils::OccupancyOptions& options, VoxelGrid<MapUtils::CellOcc>&& grid)
            : grid_(std::move(grid))         // 移动体素网格所有权
            , options_(options)              // 设置占用选项
            , accessor_(grid_.createAccessor())  // 创建新访问器
            , accessor_bound_(true)          // 标记访问器已绑定
        {}

        // 默认析构函数
        ~OccupancyMap() = default;

        // 禁止复制（地图数据结构很大）
        /**
         * @brief 删除拷贝构造函数
         */
        OccupancyMap(const OccupancyMap&) = delete;

        /**
         * @brief 删除拷贝赋值运算符
         */
        OccupancyMap& operator=(const OccupancyMap&) = delete;

        /**
         * @brief 移动构造函数
         * @param OccupancyMap&& other 要移动的源对象
         */
        OccupancyMap(OccupancyMap&& other) noexcept
            : grid_(std::move(other.grid_))  // 移动体素网格
            , options_(other.options_)       // 复制占用选项
            , update_count_(other.update_count_)  // 复制更新计数
            , miss_coords_(std::move(other.miss_coords_))  // 移动未命中坐标
            , hit_coords_(std::move(other.hit_coords_))   // 移动命中坐标
            , accessor_(grid_.createAccessor())  // 为新网格创建访问器
            , accessor_bound_(true)              // 标记访问器已绑定
        {}

        /**
         * @brief 移动赋值运算符
         * @param OccupancyMap&& other 要移动的源对象
         * @return OccupancyMap& 返回当前对象引用
         */
        OccupancyMap& operator=(OccupancyMap&& other) noexcept
        {
            if (this != &other)  // 防止自赋值
            {
                grid_ = std::move(other.grid_);  // 移动体素网格
                options_ = other.options_;       // 复制占用选项
                update_count_ = other.update_count_;  // 复制更新计数
                miss_coords_ = std::move(other.miss_coords_);  // 移动未命中坐标
                hit_coords_ = std::move(other.hit_coords_);   // 移动命中坐标
                accessor_bound_ = false;  // 标记访问器需要重新绑定
            }
            return *this;
        }

        /**
         * @brief 获取体素网格引用
         * @return VoxelGrid<MapUtils::CellOcc>& 体素网格的引用
         */
        [[nodiscard]] VoxelGrid<MapUtils::CellOcc>& getGrid()
        {
            return grid_;
        }

        /**
         * @brief 获取体素网格的常量引用
         * @return const VoxelGrid<MapUtils::CellOcc>& 体素网格的常量引用
         */
        [[nodiscard]] const VoxelGrid<MapUtils::CellOcc>& getConstGrid() const
        {
            return grid_;
        }

        /**
         * @brief 获取占用网格选项
         * @return const MapUtils::OccupancyOptions& 占用选项的常量引用
         */
        [[nodiscard]] const MapUtils::OccupancyOptions& getOptions() const
        {
            return options_;
        }

        /**
         * @brief 获取网格中激活的体素数量
         * @return size_t 激活体素数量
         */
        [[nodiscard]] size_t getActiveCellCount() const
        {
            return grid_.activeCellsCount();
        }

        /**
         * @brief 获取网格内存使用量
         * @return size_t 内存使用量（字节）
         */
        [[nodiscard]] size_t getMemoryUsage() const
        {
            return grid_.memUsage();
        }

        /**
         * @brief 设置占用网格选项
         * @param MapUtils::OccupancyOptions& options 新的占用选项
         */
        void setOptions(MapUtils::OccupancyOptions& options)
        {
            this->options_ = options;
        }

        /**
         * @brief 设置体素网格
         * @param VoxelGrid<MapUtils::CellOcc>&& grid 要移动的体素网格
         */
        void setGrid(VoxelGrid<MapUtils::CellOcc>&& grid)
        {
            this->grid_ = std::move(grid);
            accessor_bound_ = false;  // 网格改变，需要重新绑定访问器
        }

        /**
         * @brief 检查访问器是否已绑定
         * @return bool 访问器是否已绑定
         */
        [[nodiscard]] bool isAccessorBound() const
        {
            return this->accessor_bound_;
        }

        /**
         * @brief 设置访问器绑定状态
         * @param bool bound 绑定状态
         */
        void setAccessorBound(bool bound)
        {
            this->accessor_bound_ = bound;
        }

        /**
         * @brief 检查体素是否被占用
         * @details 通过访问器获取体素，比较其概率与占用阈值
         * @param Bonxai::CoordT& coord 体素坐标
         * @param Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor 访问器
         * @return bool 如果体素被占用返回true
         */
        [[nodiscard]] bool isOccupied(const Bonxai::CoordT& coord, 
                                      Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor) const
        {
            if (auto *cell = accessor.value(coord, false))  // 尝试获取体素指针
            {
                // 比较概率与阈值
                return cell->probability_log > options_.occupancy_threshold_log;
            }
            return false;  // 体素不存在，视为未被占用
        }

        /**
         * @brief 检查体素是否未知
         * @param Bonxai::CoordT& coord 体素坐标
         * @param Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor 访问器
         * @return bool 如果体素状态未知返回true
         */
        [[nodiscard]] bool isUnknown(const Bonxai::CoordT& coord, 
                                     Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor) const
        {
            if (auto *cell = accessor.value(coord, false))  // 尝试获取体素指针
            {
                // 概率等于阈值视为未知
                return cell->probability_log == options_.occupancy_threshold_log;
            }
            return true;  // 体素不存在，视为未知
        }

        /**
         * @brief 检查体素是否空闲
         * @param Bonxai::CoordT& coord 体素坐标
         * @param Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor 访问器
         * @return bool 如果体素空闲返回true
         */
        [[nodiscard]] bool isFree(const Bonxai::CoordT& coord, 
                                  Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor) const
        {
            if (auto *cell = accessor.value(coord, false))  // 尝试获取体素指针
            {
                // 概率小于阈值视为空闲
                return cell->probability_log < options_.occupancy_threshold_log;
            }
            return false;  // 体素不存在，视为未被占用
        }

        /**
         * @brief 获取所有被占用的体素坐标
         * @details 遍历所有体素，筛选出概率大于阈值的坐标
         * @param std::vector<Bonxai::CoordT>& coords 输出容器，存储占用体素坐标
         */
        void getOccupiedVoxels(std::vector<Bonxai::CoordT>& coords) const
        {
            coords.clear();  // 清空输出向量

            // 创建访问者函数对象
            auto visitor = [&](MapUtils::CellOcc &cell, const CoordT& coord)
            {
                // 检查概率是否大于占用阈值
                if (cell.probability_log > options_.occupancy_threshold_log)
                {
                    coords.push_back(coord);  // 添加到结果
                }
            };

            // 遍历所有激活的体素
            grid_.forEachCell(visitor);
        }

        /**
         * @brief 获取所有空闲的体素坐标
         * @details 遍历所有体素，筛选出概率小于阈值的坐标
         * @param std::vector<Bonxai::CoordT>& coords 输出容器，存储空闲体素坐标
         */
        void getFreeVoxels(std::vector<Bonxai::CoordT>& coords) const
        {
            coords.clear();  // 清空输出向量

            // 创建访问者函数对象
            auto visitor = [&](MapUtils::CellOcc &cell, const CoordT& coord)
            {
                // 检查概率是否小于占用阈值
                if (cell.probability_log < options_.occupancy_threshold_log)
                {
                    coords.push_back(coord);  // 添加到结果
                }
            };

            // 遍历所有激活的体素
            grid_.forEachCell(visitor);
        }

        /**
         * @brief 模板函数：获取被占用的体素点
         * @tparam PointT 点类型，需有x,y,z成员
         * @param std::vector<PointT>& points 输出容器，存储占用体素的3D坐标
         */
        template <typename PointT>
        void getOccupiedVoxels(std::vector<PointT>& points) const
        {
            thread_local std::vector<Bonxai::CoordT> coords;  // 线程局部存储，避免重复分配
            coords.clear();

            // 获取占用体素坐标
            getOccupiedVoxels(coords);
            
            // 将坐标转换为3D点
            for (const auto& coord : coords)
            {
                const auto p = grid_.coordToPos(coord);  // 体素坐标转世界坐标
                points.emplace_back(p.x, p.y, p.z);
            }
        }

        /**
         * @brief 模板函数：获取空闲的体素点
         * @tparam PointT 点类型，需有x,y,z成员
         * @param std::vector<PointT>& points 输出容器，存储空闲体素的3D坐标
         */
        template <typename PointT>
        void getFreeVoxels(std::vector<PointT>& points) const
        {
            thread_local std::vector<Bonxai::CoordT> coords;  // 线程局部存储，避免重复分配
            coords.clear();

            // 获取空闲体素坐标
            getFreeVoxels(coords);
            
            // 将坐标转换为3D点
            for (const auto& coord : coords)
            {
                const auto p = grid_.coordToPos(coord);  // 体素坐标转世界坐标
                points.emplace_back(p.x, p.y, p.z);
            }
        }

        /**
         * @brief 添加命中点（使用外部访问器）
         * @details 激光扫描到的点，表示障碍物存在
         *          增加该体素的占用概率
         * @param Vector3D& point 世界坐标系中的3D点
         * @param Accessor& accessor 外部提供的访问器
         */
        void addHitPoint(const Vector3D& point, Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor)
        {
            const auto coord = grid_.posToCoord(point);  // 将3D点转换为体素坐标
            MapUtils::CellOcc* cell = accessor.value(coord, true);  // 获取或创建体素

            // 检查是否需要更新（避免同一周期重复更新）
            if (cell->update_id != update_count_)
            {
                // 增加占用概率，限制不超过最大值
                cell->probability_log = std::min(cell->probability_log + options_.prob_hit_log,
                                                options_.clamp_max_log);
                
                cell->update_id = update_count_;  // 标记当前更新周期
                hit_coords_.push_back(coord);     // 记录命中坐标
            }
        }
        
        /**
         * @brief 添加命中点（使用外部访问器，体素坐标版本）
         * @param Bonxai::CoordT& coord 体素坐标
         * @param Accessor& accessor 外部提供的访问器
         */
        void addHitPoint(const Bonxai::CoordT& coord, Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor)
        {
            MapUtils::CellOcc* cell = accessor.value(coord, true);  // 获取或创建体素

            if (cell->update_id != update_count_)  // 避免重复更新
            {
                // 增加占用概率
                cell->probability_log = std::min(cell->probability_log + options_.prob_hit_log,
                                                options_.clamp_max_log);
                
                cell->update_id = update_count_;  // 标记当前更新周期
                hit_coords_.push_back(coord);     // 记录命中坐标
            }
        }

        /**
         * @brief 添加命中点（使用内部访问器）
         * @param Vector3D& point 世界坐标系中的3D点
         */
        void addHitPoint(const Vector3D& point)
        {
            const auto coord = grid_.posToCoord(point);  // 将3D点转换为体素坐标
            MapUtils::CellOcc* cell = accessor_.value(coord, true);  // 使用内部访问器

            if (cell->update_id != update_count_)  // 避免重复更新
            {
                // 增加占用概率
                cell->probability_log = std::min(cell->probability_log + options_.prob_hit_log,
                                                options_.clamp_max_log);
                
                cell->update_id = update_count_;  // 标记当前更新周期
                hit_coords_.push_back(coord);     // 记录命中坐标
            }
        }
        
        /**
         * @brief 添加命中点（使用内部访问器，体素坐标版本）
         * @param Bonxai::CoordT& coord 体素坐标
         */
        void addHitPoint(const Bonxai::CoordT& coord)
        {
            MapUtils::CellOcc* cell = accessor_.value(coord, true);  // 使用内部访问器

            if (cell->update_id != update_count_)  // 避免重复更新
            {
                // 增加占用概率
                cell->probability_log = std::min(cell->probability_log + options_.prob_hit_log,
                                                options_.clamp_max_log);
                
                cell->update_id = update_count_;  // 标记当前更新周期
                hit_coords_.push_back(coord);     // 记录命中坐标
            }
        }

        /**
         * @brief 添加未命中点（使用外部访问器）
         * @details 激光扫描路径上的点，表示自由空间
         *          减少该体素的占用概率
         * @param Vector3D& point 世界坐标系中的3D点
         * @param Accessor& accessor 外部提供的访问器
         */
        void addMissPoint(const Vector3D& point, Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor)
        {
            const auto coord = grid_.posToCoord(point);  // 将3D点转换为体素坐标
            MapUtils::CellOcc* cell = accessor.value(coord, true);  // 获取或创建体素

            if (cell->update_id != update_count_)  // 避免重复更新
            {
                // 减少占用概率，限制不小于最小值
                cell->probability_log = std::max(cell->probability_log + options_.prob_miss_log,
                                                options_.clamp_min_log);
            }

            cell->update_id = update_count_;  // 标记当前更新周期
            miss_coords_.push_back(coord);    // 记录未命中坐标
        }

        /**
         * @brief 添加未命中点（使用外部访问器，体素坐标版本）
         * @param CoordT& coord 体素坐标
         * @param Accessor& accessor 外部提供的访问器
         */
        void addMissPoint(const Bonxai::CoordT& coord, Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor& accessor)
        {
            MapUtils::CellOcc* cell = accessor.value(coord, true);  // 获取或创建体素

            if (cell->update_id != update_count_)  // 避免重复更新
            {
                // 减少占用概率
                cell->probability_log = std::max(cell->probability_log + options_.prob_miss_log,
                                                options_.clamp_min_log);
            }

            cell->update_id = update_count_;  // 标记当前更新周期
            miss_coords_.push_back(coord);    // 记录未命中坐标
        }

        /**
         * @brief 添加未命中点（使用内部访问器）
         * @param Vector3D& point 世界坐标系中的3D点
         */
        void addMissPoint(const Vector3D& point)
        {
            const auto coord = grid_.posToCoord(point);  // 将3D点转换为体素坐标
            MapUtils::CellOcc* cell = accessor_.value(coord, true);  // 使用内部访问器

            if (cell->update_id != update_count_)  // 避免重复更新
            {
                // 减少占用概率
                cell->probability_log = std::max(cell->probability_log + options_.prob_miss_log,
                                                options_.clamp_min_log);
            }

            cell->update_id = update_count_;  // 标记当前更新周期
            miss_coords_.push_back(coord);    // 记录未命中坐标
        }

        /**
         * @brief 添加未命中点（使用内部访问器，体素坐标版本）
         * @param Vector3D& point 世界坐标系中的3D点
         */
        void addMissPoint(const Bonxai::CoordT& coord)
        {
            MapUtils::CellOcc* cell = accessor_.value(coord, true);  // 使用内部访问器

            if (cell->update_id != update_count_)  // 避免重复更新
            {
                // 减少占用概率
                cell->probability_log = std::max(cell->probability_log + options_.prob_miss_log,
                                                options_.clamp_min_log);
            }

            cell->update_id = update_count_;  // 标记当前更新周期
            miss_coords_.push_back(coord);    // 记录未命中坐标
        }

        /**
         * @brief 增加更新计数
         * @details 更新计数在1-3之间循环，用于标识当前更新周期
         *          避免同一周期内重复更新同一体素
         */
        void incrementUpdateCount()
        {
            if (++this->update_count_ == 4)  // 循环检查
            {
                this->update_count_ = 1;  // 回到1
            }
        }

        /**
         * @brief 主更新函数，向用户公开，用于更新点云
         * @details 处理单帧激光点云数据
         *          1. 对每个点，判断是否超出最大范围
         *          2. 对范围内的点标记为命中
         *          3. 对超出范围的点沿射线方向截断
         *          4. 更新射线上的所有体素为自由空间
         * @tparam PointT 点类型
         * @tparam Allocator 分配器类型
         * @param std::vector<PointT, Allocator>& points 点云数据
         * @param PointT& origin 传感器原点在世界坐标系中的位置
         * @param double max_range 传感器最大检测范围
         */
        template <typename PointT, typename Allocator>
        void insertPointCloud(const std::vector<PointT, Allocator>& points, 
                            const PointT& origin, double max_range)
        {
            const auto from_point = ConvertPoint<Vector3D>(origin);
            const double max_range_sq = max_range * max_range;
            
            auto accessor = grid_.createAccessor();
            
            // ========== 新增：膨胀参数 ==========
            double inflation_radius = options_.inflation_radius;  // 膨胀半径，建议作为参数
            double resolution = options_.resolution;
            int inflation_step = 0;
            
            if (inflation_radius > 0) {
                inflation_step = static_cast<int>(std::ceil(inflation_radius / resolution));
            }
            
            // 存储原始命中点，用于后续膨胀
            std::vector<CoordT> original_hit_coords;
            original_hit_coords.reserve(points.size());
            
            // 遍历点云中的每个点
            for (const auto& point : points)
            {
                const auto to_point = ConvertPoint<Vector3D>(point);
                
                Vector3D vector_from_to(to_point - from_point);
                const double squared_norm = vector_from_to.squaredNorm();
                Vector3D vector_from_to_unit = vector_from_to / std::sqrt(squared_norm);
                
                if (squared_norm >= max_range_sq)
                {
                    const Vector3D new_to_point = from_point + (vector_from_to_unit * max_range);
                    addMissPoint(new_to_point, accessor);
                }
                else
                {
                    // 保存原始命中点坐标
                    const auto coord = grid_.posToCoord(to_point);
                    original_hit_coords.push_back(coord);
                    
                    // 标记为命中
                    addHitPoint(to_point, accessor);
                }
            }
            
            // ========== 新增：膨胀处理 ==========
            if (inflation_step > 0 && !original_hit_coords.empty()) {
                // 对每个原始命中点进行膨胀
                for (const auto& center_coord : original_hit_coords) {
                    // 遍历膨胀区域
                    for (int dx = -inflation_step; dx <= inflation_step; ++dx) {
                        for (int dy = -inflation_step; dy <= inflation_step; ++dy) {
                            // 计算实际距离
                            double distance = std::sqrt(dx*dx + dy*dy) * resolution;
                            if (distance <= inflation_radius) {
                                // 创建膨胀坐标（保持Z不变）
                                CoordT inflated_coord;
                                inflated_coord.x = center_coord.x + dx;
                                inflated_coord.y = center_coord.y + dy;
                                inflated_coord.z = center_coord.z;  // Z轴不变
                                
                                // 获取或创建体素
                                MapUtils::CellOcc* cell = accessor.value(inflated_coord, true);
                                
                                // 检查是否需要更新
                                if (cell->update_id != update_count_) {
                                    // 设置膨胀区域的概率（比原始障碍物概率稍低）
                                    int32_t inflated_prob = options_.clamp_max_log * 0.9;
                                    cell->probability_log = std::min(inflated_prob, 
                                                                options_.clamp_max_log);
                                    cell->update_id = update_count_;
                                    
                                    // 记录到hit_coords_中，这样射线也会被清理
                                    hit_coords_.push_back(inflated_coord);
                                }
                            }
                        }
                    }
                }
            }
            
            // 更新射线上的所有体素为自由空间
            updateFreeCells(from_point);
        }
        
    private:
        /**
         * @brief 更新从原点到所有命中/未命中点的射线上的体素
         * @details 核心函数：更新传感器视线上的体素为自由空间
         *          对每个命中/未命中点，从原点到该点之间的射线上的体素
         *          都被标记为未命中（自由空间）
         * @param Vector3D& origin 传感器原点
         */
        void updateFreeCells(const Vector3D& origin)
        {
            // 创建新的访问器用于本次更新
            auto accessor = grid_.createAccessor();

            // 清理函数：将射线上的体素标记为未命中
            auto clearPoint = [this, &accessor](const CoordT& coord)
            {
                MapUtils::CellOcc* cell = accessor.value(coord, true);  // 获取或创建体素
                if (cell->update_id != update_count_)  // 避免重复更新
                {
                    // 减少占用概率，标记为自由空间
                    cell->probability_log = std::max(cell->probability_log + options_.prob_miss_log, 
                                                     options_.clamp_min_log);
                    cell->update_id = update_count_;  // 标记当前更新周期
                }
                return true;  // 继续遍历射线
            };

            const auto coord_origin = grid_.posToCoord(origin);  // 转换原点到体素坐标

            // 处理所有命中点对应的射线
            for (const auto& coord_end_hit : hit_coords_)
            {
                // 遍历从原点到命中点之间的所有体素
                MapUtils::RayIterator(coord_origin, coord_end_hit, clearPoint);
            }
            hit_coords_.clear();  // 清空命中点记录

            // 处理所有未命中点对应的射线
            for (const auto& coord_end_miss : miss_coords_)
            {
                // 遍历从原点到未命中点之间的所有体素
                MapUtils::RayIterator(coord_origin, coord_end_miss, clearPoint);
            }
            miss_coords_.clear();  // 清空未命中点记录

            // 更新循环计数
            if (++update_count_ == 4)
            {
                update_count_ = 1;  // 回到1，形成1-3的循环
            }
        }
        
        // 主数据结构：存储占用网格的稀疏体素网格
        VoxelGrid<MapUtils::CellOcc> grid_;

        // 占用网格的配置选项
        MapUtils::OccupancyOptions options_;
        
        // 更新计数，在1-3之间循环，用于避免同一周期重复更新
        uint8_t update_count_ = 1;

        // 存储超出最大范围的未命中点坐标
        std::vector<CoordT> miss_coords_;

        // 存储命中的体素坐标（障碍物表面）
        std::vector<CoordT> hit_coords_;

        // 网格访问器，尽可能复用以减少创建开销
        mutable Bonxai::VoxelGrid<MapUtils::CellOcc>::Accessor accessor_;

        // 访问器绑定状态标志
        bool accessor_bound_ {false};
    };
}  // namespace Bonxai