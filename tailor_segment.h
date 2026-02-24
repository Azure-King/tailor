#pragma once
#include "tailor_concept.h"
#include "tailor_point.h"
#include <algorithm>
#include <assert.h>
#include <cmath>

TAILOR_NAMESPACE_BEGIN

template<class UserData = void>
class DataBase {
	UserData data;

public:
	template<class... Args>
	DataBase(Args&&... args) :data(std::forward<Args>(args)...) {
	}
	UserData& Data() { return data; }
	const UserData& Data() const { return data; }
};

/**
 * @brief 默认使用不携带用户数据的全特化版本, 空基类优化
 */
template<>
class DataBase<void> {};

template<class PType, class UserData = void>
class LineSegment :public DataBase<UserData> {
public:
	using PointType = PType;
	using UserDataType = UserData;
private:
	PointType a;
	PointType b;
public:
	template<class... Args>
	constexpr LineSegment(const PointType& a, const PointType& b, Args&&... args) :
		DataBase<UserData>(std::forward<Args>(args)...), a(a), b(b) {
	}

	PointType& Point0() { return a; }
	PointType& Point1() { return b; }
	const PointType& Point0() const { return a; }
	const PointType& Point1() const { return b; }
};

/**
 * @brief 直线段特征提取
 * @note 当前模板实现默认为tailor::LineSegment<Point, Any>, 如需支持其他直线段类型, 需要进行特化
 *		 LineSegmentTraits<SegmentType> 要求:
 *			1. 公开 PointType 及 CoordinateType
 *			2. 定义 GetPoint0 和 GetPoint1 方法, 它们分别代表直线段的始末点
 *			3. 定义 Construct(PointType, PointType, CurveType) 方法, 其中 CurveType 本次构建的边的源边,
 *			   算法不会凭空构建边，仅会将边分裂为多条边, 如果线段内包含无法复制的数据, 则该函数大概率需要重写
 * @tparam Segment 直线段类型
 */
template <typename Segment>
struct LineSegmentTraits {
	using CurveType = Segment;
	using PointType = typename CurveType::PointType;
	using CoordinateType = typename PointTraits<PointType>::CoordinateType;

	static constexpr PointType GetPoint0(const CurveType& curve) { return curve.Point0(); }
	static constexpr PointType GetPoint1(const CurveType& curve) { return curve.Point1(); }

	static constexpr CurveType Construct(const PointType& p0, const PointType& p1, const CurveType& from) {
		if constexpr (std::is_same_v<typename CurveType::UserDataType, void>) {
			return CurveType(p0, p1);
		} else {
			return CurveType(p0, p1, from.Data());
		}
	}
};

template<size_t precision>
struct PrecisionCore {
	static constexpr double Epsilon(size_t preci) {
		double result = 1.0;
		while (preci) {
			result /= 10.0;
			preci--;
		}
		return result;
	};

	static constexpr double VALUE_EPSILON = Epsilon(precision);
	static constexpr double POINT_EPSILON = Epsilon(precision);
	static constexpr double ANGLE_EPSILON = Epsilon(precision);
};

/**
 * @brief  该类包含一些直线段相关的计算方法, 模板实现依赖 LineSegmentTraits<Segment>
 * @tparam LineSegmentType
 * @tparam Precision
 */
template<class Segment>
class LineSegmentUtils {
public:
	using CurveType = Segment;
	using PointType = typename LineSegmentTraits<CurveType>::PointType;
	using CoordinateType = typename PointTraits<PointType>::CoordinateType;

	LineSegmentUtils() = default;

	template<class PU, class ST>
	LineSegmentUtils(PU&& pu, ST&& st) :pUtils(std::forward<PU>(pu), std::forward<ST>(st)) {
	}

	PointType Project(const PointType& p, const CurveType& line) const {
		decltype(auto) p0 = cTraits.GetPoint0(line);
		decltype(auto) p1 = cTraits.GetPoint1(line);
		const auto u = ProjectU(p, p0, p1);
		auto v = static_cast<CoordinateType>(1) - u;
		return PointType(v * pUtils.X(p0) + u * pUtils.X(p1), v * pUtils.Y(p0) + u * pUtils.Y(p1));
	}

	CurveType ConstructCurve(const PointType& a, const PointType& b,
		const CurveType& from) const {
		assert(!pUtils.IsSamePosition(a, b, 1e-10));
		return cTraits.Construct(a, b, from);
	}

	bool IsLineSegmentCross(const PointType& A, const PointType& B,
		const PointType& C, const PointType& D, CoordinateType tolerance) const {
		auto a = pUtils.Cross(pUtils.Sub(C, A), pUtils.Sub(C, D));
		auto b = pUtils.Cross(pUtils.Sub(C, B), pUtils.Sub(C, D));
		auto c = pUtils.Cross(pUtils.Sub(A, C), pUtils.Sub(A, B));
		auto d = pUtils.Cross(pUtils.Sub(A, D), pUtils.Sub(A, B));

		// TODO 精度问题
		return
			pUtils.Cross(pUtils.Sub(C, A), pUtils.Sub(C, D)) * pUtils.Cross(pUtils.Sub(C, B), pUtils.Sub(C, D)) <= tolerance &&
			pUtils.Cross(pUtils.Sub(A, C), pUtils.Sub(A, B)) * pUtils.Cross(pUtils.Sub(A, D), pUtils.Sub(A, B)) <= tolerance;
	}

	// 调用该函数之前, 必须调用 IsLineSegmentCross
	std::optional<PointType> GetCrossPoint(const PointType& A, const PointType& B, const PointType& C, const PointType& D, CoordinateType tolerance) const {
		if (!IsLineSegmentCross(A, B, C, D, tolerance)) {
			// 当点在线段上时，该函数判断不准
			return {};
		}

		auto base = pUtils.Sub(D, C);
		using namespace std;
		auto d1 = fabs(pUtils.Cross(base, pUtils.Sub(A, C)));
		auto d2 = fabs(pUtils.Cross(base, pUtils.Sub(B, C)));
		auto t = d1 / (d1 + d2);
		return pUtils.Add(A, pUtils.Mult(pUtils.Sub(B, A), t));
	}

private:
	CoordinateType ProjectU(const PointType& p, const PointType& a, const PointType& b) const {
		auto ab_x = pUtils.X(b) - pUtils.X(a);
		auto ab_y = pUtils.Y(b) - pUtils.Y(a);
		auto ap_x = pUtils.X(p) - pUtils.X(a);
		auto ap_y = pUtils.Y(p) - pUtils.Y(a);
		auto v = ab_x * ap_x + ab_y * ap_y;

		if (v <= static_cast<CoordinateType>(0)) {
			return static_cast<CoordinateType>(0);
		}

		auto length_sq = ab_x * ab_x + ab_y * ab_y;
		if (v >= length_sq) {
			return static_cast<CoordinateType>(1);
		}

		return v / length_sq;
	}

private:
	using PUtils = PointUtils<PointType>;
	using CTraits = LineSegmentTraits<CurveType>;

	PUtils pUtils{};
	CTraits cTraits{};
};

template<class LineSegmentType, class PC>
class LineSegmentAnalyserCore :
	public PointUtils<typename LineSegmentTraits<LineSegmentType>::PointType>,
	public LineSegmentUtils<LineSegmentType> {
	using Super0 = PointUtils<typename LineSegmentTraits<LineSegmentType>::PointType>;
	using Super1 = LineSegmentUtils<LineSegmentType>;
public:
	using Precision = PC;
	LineSegmentAnalyserCore() = default;

	template<class... Args>
	auto GetCrossPoint(Args&&... args) const {
		return Super1::GetCrossPoint(std::forward<Args>(args)..., Precision::VALUE_EPSILON);
	}

	template<class... Args>
	auto DirectionFromTo(Args&&... args) const {
		return Super0::DirectionFromTo(std::forward<Args>(args)..., Precision::VALUE_EPSILON);
	}
};

template<class LineSegmentType, class Core>
class LineSegmentAnalyser {
private:
	static constexpr size_t AI_index = static_cast<size_t>(PieceType::AI);
	static constexpr size_t IB_index = static_cast<size_t>(PieceType::IB);
	static constexpr size_t CI_index = static_cast<size_t>(PieceType::CI);
	static constexpr size_t ID_index = static_cast<size_t>(PieceType::ID);

	using enum tailor::CurveRelativePositionType;
	using enum tailor::VertexRelativePositionType;

public:
	using PointType = typename LineSegmentType::PointType;
	using CoordinateType = typename PointTraits<PointType>::CoordinateType;
	using CurveType = LineSegmentType;
	using UserDataType = typename CurveType::UserDataType;

	Core core;

	static const PointType& Start(const CurveType& curve) { return curve.Point0(); }
	static const PointType& End(const CurveType& curve) { return curve.Point1(); }
	static CurveType Reverse(const CurveType& curve) {
		if constexpr (std::is_same_v<UserDataType, void>) {
			return CurveType(curve.Point1(), curve.Point0());
		} else {
			return CurveType(curve.Point1(), curve.Point0(), curve.Data());
		}
	}
	VertexRelativePositionType CalcateVertexRelativePosition(const PointType& a, const PointType& b) const {
		//vertex_run_times++;
		return core.DirectionFromTo(a, b);
	}

	CurveRelativePositionResult2<CurveType> CalcateEdgeRelativePosition(const CurveType& ab, const CurveType& cd,
		tailor::OnlyFocusOnRelativePositionAndCoincidenceDetails) const {
		const auto& A = ab.Point0();
		const auto& B = ab.Point1();
		const auto& C = cd.Point0();
		const auto& D = cd.Point1();

		assert(CalcateVertexRelativePosition(A, C) == VertexRelativePositionType::Same);

		CurveRelativePositionResult2<CurveType> result{};

		// 如果 B 和 D 重合, 则两条边重合
		if (CalcateVertexRelativePosition(B, D) == VertexRelativePositionType::Same) TAILOR_UNLIKELY{
			result.positionType = CurveRelativePositionType::Coincident;
			result.edges[AI_index] = ab;
			result.edges[CI_index] = cd;
			return result;
		}

			// 如果 B 在 CD 上
			if (IsOnEdge(B, cd)) TAILOR_UNLIKELY{
				result.positionType = CurveRelativePositionType::Coincident;
				result.edges[AI_index] = ab;

				auto split_result = SplitEdge(cd, B);
				result.edges[CI_index] = std::move(split_result.GetPiece(AI)); // move 目前没什么用
				result.edges[ID_index] = std::move(split_result.GetPiece(IB));
				return result;
			}

				// 如果 D 在 AB 上
				if (IsOnEdge(D, ab)) TAILOR_UNLIKELY{
					result.positionType = CurveRelativePositionType::Coincident;
					result.edges[CI_index] = cd;

					auto split_result = SplitEdge(ab, D);
					result.edges[AI_index] = std::move(split_result.GetPiece(AI)); // move 目前没什么用
					result.edges[IB_index] = std::move(split_result.GetPiece(IB));
					return result;
				}

		auto AB = core.Sub(B, A);
		auto CD = core.Sub(D, C);

		// ??? 此处 x 有没有可能为很小的负数, 此时曲线单调性会判断为 Top
		auto ab_k = Y(AB) / X(AB);
		auto cd_k = Y(CD) / X(CD);

		// 比较斜率大小
		if (ab_k > cd_k) {
			result.positionType = CurveRelativePositionType::Downward;
		} else {
			result.positionType = CurveRelativePositionType::Upward;
		}
		return result;
	}

	CurveRelativePositionResult2<CurveType> CalcateEdgeRelativePosition(const CurveType& ab, const CurveType& cd,
		tailor::OnlyFocusOnRelativePositionWithoutCoincidence) const {
		const auto& A = ab.Point0();
		const auto& B = ab.Point1();
		const auto& C = cd.Point0();
		const auto& D = cd.Point1();

		CurveRelativePositionResult2<CurveType> result{};

		// AB 竖直向上, 由于两曲线不重合, 所以结果必定是 Downward
		if (CalcateVertexRelativePosition(A, B) == VertexRelativePositionType::Top) TAILOR_UNLIKELY{
			result.positionType = CurveRelativePositionType::Downward;
			return result;
		}

			if (SampleInX(A, B, C) > Y(C)) {
				result.positionType = CurveRelativePositionType::Downward;
				return result;
			} else {
				result.positionType = CurveRelativePositionType::Upward;
				return result;
			}
	}

	CurveRelativePositionResult2<CurveType> CalcateEdgeRelativePosition(const CurveType& ab, const CurveType& cd,
		tailor::OnlyFocusOnRelativePositionDetailsWithoutCoincidence) const {
		const auto& A = ab.Point0();
		const auto& B = ab.Point1();
		const auto& C = cd.Point0();
		const auto& D = cd.Point1();

		CurveRelativePositionResult2<CurveType> result{};

		if (CalcateVertexRelativePosition(A, C) == VertexRelativePositionType::Same) {
			auto AB = core.Sub(B, A);
			auto CD = core.Sub(D, C);

			// ??? 此处 x 有没有可能为很小的负数, 此时曲线单调性会判断为 Top
			auto ab_k = Y(AB) / X(AB);
			auto cd_k = Y(CD) / X(CD);

			// 比较斜率大小
			if (ab_k > cd_k) {
				result.positionType = CurveRelativePositionType::Downward;
			} else {
				result.positionType = CurveRelativePositionType::Upward;
			}

			result.edges[AI_index] = ab;
			result.edges[CI_index] = cd;
			return result;
		}

		// 如果 B 在 CD 上
		if (IsOnEdge(B, cd)) TAILOR_UNLIKELY{
			result.edges[AI_index] = ab;
			auto split_result = SplitEdge(cd, B);
			if (split_result.HasPiece(AI)) result.edges[CI_index] = split_result.GetPiece(AI);
			if (split_result.HasPiece(IB)) result.edges[ID_index] = split_result.GetPiece(IB);
			result.positionType = (SampleInX(A, B, C) > Y(C)) ? Downward : Upward;
			return result;
		}

			// 如果 D 在 AB 上
			if (IsOnEdge(D, ab)) TAILOR_UNLIKELY{
				result.edges[CI_index] = cd;
				auto split_result = SplitEdge(ab, D);
				if (split_result.HasPiece(AI)) result.edges[AI_index] = split_result.GetPiece(AI);
				if (split_result.HasPiece(IB)) result.edges[IB_index] = split_result.GetPiece(IB);
				result.positionType = (SampleInX(A, B, C) > Y(C)) ? Downward : Upward;
				return result;
			}

		auto ip = core.GetCrossPoint(A, B, C, D);

		if (!ip.has_value()) {
			result.edges[AI_index] = ab;
			result.edges[CI_index] = cd;
			result.positionType = (SampleInX(A, B, C) > Y(C)) ? Downward : Upward;
			return result;
		}
		const auto& I = *ip;

		result.edges[AI_index] = core.ConstructCurve(A, I, ab);
		result.edges[IB_index] = core.ConstructCurve(I, B, ab);
		result.edges[CI_index] = core.ConstructCurve(C, I, cd);
		result.edges[ID_index] = core.ConstructCurve(I, D, cd);
		result.positionType = (SampleInX(A, B, C) > Y(C)) ? Downward : Upward;

		return result;
	}

	bool IsOnEdge(const PointType& p, const CurveType& seg) const {
		return CalcateVertexRelativePosition(core.Project(p, seg), p) == VertexRelativePositionType::Same;
	}

	SplitEdgeResult<CurveType> SplitEdge(const CurveType& seg, const PointType& p) const {
		assert(IsOnEdge(p, seg));
		SplitEdgeResult<CurveType> result{};

		const auto& A = seg.Point0();
		if (CalcateVertexRelativePosition(A, p) != VertexRelativePositionType::Same) {
			result.edges[static_cast<size_t>(PieceType::AI)] = core.ConstructCurve(A, p, seg);
		}
		const auto& B = seg.Point1();
		if (CalcateVertexRelativePosition(p, B) != VertexRelativePositionType::Same) {
			result.edges[static_cast<size_t>(PieceType::IB)] = core.ConstructCurve(p, B, seg);
		}

		return result;
	}

	MonotonicSplitResult<CurveType> SplitToMonotonic(const CurveType& edge) {
		return MonotonicSplitResult<CurveType>{ edge };
	}
private:
	auto SampleInX(const PointType& a, const PointType& b, const PointType& p) const {
		auto ab = core.Sub(b, a);
		return Y(a) + Y(ab) / X(ab) * (X(p) - X(a));
	}

	decltype(auto) X(const PointType& p) const {
		return core.X(p);
	}
	decltype(auto) Y(const PointType& p) const {
		return core.Y(p);
	}

	// A <= C < B && C < D
	CurveRelativePositionType CalcCurveRelativePosition(const CurveType& ab, const CurveType& cd) const {
		const auto& A = ab.Point0();
		const auto& B = ab.Point1();
		const auto& C = cd.Point0();
		const auto& D = cd.Point1();

		double xmin = X(C);
		using namespace std;
		double xmax = min(X(B), X(D));

		double xmid = (xmin + xmax) / 2.0;

		auto AB_rpt = CalcateVertexRelativePosition(A, B);
		auto CD_rpt = CalcateVertexRelativePosition(C, D);

		auto y_in_ab = 0.0;
		auto y_in_cd = 0.0;

		if (AB_rpt == VertexRelativePositionType::Top) {
			y_in_ab = (Y(A) + Y(B)) / 2.0;
		} else {
			y_in_ab = Y(B) - (Y(B) - Y(A)) * (X(B) - xmid) / (X(B) - X(A));
		}

		if (CD_rpt == VertexRelativePositionType::Top) {
			y_in_cd = (Y(C) + Y(D)) / 2.0;
		} else {
			y_in_cd = Y(D) - (Y(D) - Y(C)) * (X(D) - xmid) / (X(D) - X(C));
		}

		return (y_in_ab < y_in_cd) ?
			CurveRelativePositionType::Upward :
			CurveRelativePositionType::Downward;
	}
};

TAILOR_NAMESPACE_END