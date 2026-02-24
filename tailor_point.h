#pragma once

namespace tailor {
template<typename T>
struct Point {
	using CoordinateType = T;
	T x;
	T y;
};

/**
 * @brief 点特征提取
 * @note 当前模板实现默认为tailor::Point<T>, 如需支持其他点类型, 需要进行特化
 *		 PointTraits<PointType> 要求:
 *			1. 公开CoordinateType
 *			2. 定义 GetX 和 GetY 方法, 以获取点的坐标值
 *			3. 定义 Construct 方法, 以创建点
 *			4. 方法可不为 static
 * @tparam PointType 点类型
 */
template <typename PointType>
struct PointTraits {
	using CoordinateType = typename PointType::CoordinateType;

	static constexpr CoordinateType GetX(const PointType& point) { return point.x; }
	static constexpr CoordinateType GetY(const PointType& point) { return point.y; }

	static constexpr PointType ConstructPoint(CoordinateType x, CoordinateType y) {
		return PointType{ x, y };
	}
};

template <typename PointType>
class PointUtils {
public:
	using CoordinateType = typename PointType::CoordinateType;

	PointUtils() = default;

	template<class... Args>
	PointUtils(Args&&... args) : traits(std::forward<Args>(args)...) {
	}

	auto ConstructPoint(CoordinateType x, CoordinateType y) const {
		return traits.ConstructPoint(x, y);
	}

	bool IsSamePosition(const PointType& a, const PointType& b, CoordinateType tolerance) const {
		using std::abs;
		return abs(X(a) - X(b)) < tolerance
			&& abs(Y(a) - Y(b)) < tolerance;
	}

	auto DirectionFromTo(const PointType& a, const PointType& b, CoordinateType tolerance) const {
		return DirectionFromToImpl(X(b) - X(a), Y(b) - Y(a), tolerance);
	}

	auto X(const PointType& a) const { return traits.GetX(a); }
	auto Y(const PointType& a) const { return traits.GetY(a); }
	auto Sub(const PointType& a, const PointType& b) const {
		return traits.ConstructPoint(X(a) - X(b), Y(a) - Y(b));
	}
	auto Add(const PointType& a, const PointType& b) const {
		return traits.ConstructPoint(X(a) + X(b), Y(a) + Y(b));
	}
	auto Dot(const PointType& a, const PointType& b) const {
		return X(a) * X(b) + Y(a) * Y(b);
	}
	auto Cross(const PointType& a, const PointType& b) const {
		return X(a) * Y(b) - Y(a) * X(b);
	}
	auto SqLen(const PointType& a) const { return Dot(a, a); }
	auto Len(const PointType& a)const {
		using namespace std;
		return sqrt(SqLen(a));
	}
	auto Normlize(const PointType& a) const {
		auto len = Len(a);
		return Divide(a, len);
	}
	auto Mult(const PointType& a, CoordinateType v) const {
		return traits.ConstructPoint(v * X(a), v * Y(a));
	}
	auto Divide(const PointType& a, CoordinateType v) const {
		return traits.ConstructPoint(X(a) / v, Y(a) / v);
	}

private:
	static VertexRelativePositionType DirectionFromToImpl(CoordinateType dx, CoordinateType dy, CoordinateType tolerance) {
		using std::abs;
		int x = (abs(dx) < tolerance) ? 0 : (dx < 0 ?
			static_cast<int>(VertexRelativePositionType::Left) :
			static_cast<int>(VertexRelativePositionType::Right));
		int y = (abs(dy) < tolerance) ? 0 : (dy < 0 ?
			static_cast<int>(VertexRelativePositionType::Bottom) :
			static_cast<int>(VertexRelativePositionType::Top));
		return static_cast<VertexRelativePositionType>(x + y);
	}
private:
	PointTraits<PointType> traits;
};
}
