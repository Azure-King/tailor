#pragma once

#include <concepts>
#include <optional>
#include <stdexcept>
#include <type_traits>

#define TAILOR_NAMESPACE_BEGIN namespace tailor {
#define TAILOR_NAMESPACE_END }

#define TAILOR_NOINLINE __declspec(noinline)
#define TAILOR_UNLIKELY [[unlikely]]
//#define TAILOR_UNLIKELY

TAILOR_NAMESPACE_BEGIN

// 仅关注相对位置和重合详情
// 仅当曲线前端重合时，必须给出AI、IC、ID详细信息
// 其他情况不关心
// 附加条件： A == C
struct OnlyFocusOnRelativePositionAndCoincidenceDetails {};

// 仅关注相对位置，重合情形在之前已经被处理了，所以不需要关心
// 无需给出AI、BI、IC、ID详细信息
struct OnlyFocusOnRelativePositionWithoutCoincidence {};

// 关注相对位置详情，重合情形在之前已经被处理了，所以不需要关心
// 必须给出AI、BI、IC、ID详细信息
struct OnlyFocusOnRelativePositionDetailsWithoutCoincidence {};

using Int = int32_t;
using Handle = uint32_t;
using Size = uint32_t;

static constexpr Handle npos = static_cast<Handle>(-1);

template<class T>
using Optional = std::optional<T>;

enum class PieceType {
	AI = 0,
	IB = 1,
	CI = 2,
	ID = 3,
	//AB = 0,
	//CD = 2,
};

template<PieceType piece>
struct Piece {
};

constexpr auto AI = Piece<PieceType::AI>{};
constexpr auto IB = Piece<PieceType::IB>{};
constexpr auto CI = Piece<PieceType::CI>{};
constexpr auto ID = Piece<PieceType::ID>{};

/**
 * @brief 边概念
 */
template<class Edge>
concept EdgeConcept = requires(Edge a, Edge b) {
	a = b;
	a = std::move(b);
	Edge(a);
	Edge(std::move(b));
};

/**
 * @brief 点概念
 */
template<class Vertex>
concept VertexConcept = requires(Vertex a, Vertex b) {
	a = b;
	a = std::move(b);
	Vertex(a);
	Vertex(std::move(b));
};

/**
 * @brief 点相对位置关系
 */
enum class VertexRelativePositionType {
	Top = +1,
	Bottom = -1,
	Left = -3,
	Right = +3,
	LeftTop = Left + Top,
	LeftBottom = Left + Bottom,
	RightTop = Right + Top,
	RightBottom = Right + Bottom,
	Same = Top + Bottom + Left + Right,
};

/**
 * @brief 计算点a和点b的相对位置关系，或是计算向量ab的方向
 * @tparam T 数值类型
 * @param dx ab.x
 * @param dy ab.y
 * @return   VertexRelativePositionType结果，即：点b在点a的什么方位，或是向量ab的方向如何
 */
template<class T>
VertexRelativePositionType CalcVectorType(T dx, T dy) {
	int x = (0 == dx) ? 0 : (dx < 0 ? -1 : 1);
	int y = (0 == dy) ? 0 : (dy < 0 ? -3 : 3);
	return static_cast<VertexRelativePositionType>(x + y);
}

/**
 * @brief 边相对位置关系
 */
enum class CurveRelativePositionType {
	Upward = 1,
	Downward = -1,
	Coincident = 0,
};

/**
 * @brief 边的单调分割结果
 */
template<EdgeConcept Edge>
struct MonotonicSplitResult {
	Edge monotonous;
	Optional<Edge> remaining;
};

/**
 * @brief 边的相对位置关系结果 ×
 */
template <EdgeConcept Edge>
struct CurveRelativePositionResult {
	using CurveSplitResult = MonotonicSplitResult<Edge>;

	CurveSplitResult a;
	CurveSplitResult b;
	CurveRelativePositionType relativePosition; // b 在 a 的什么位置
};

template <EdgeConcept Edge>
struct CurveRelativePositionResult2 {
	template<PieceType index>
	bool HasPiece(Piece<index>) const {
		return edges[static_cast<size_t>(index)].has_value();
	}

	template<PieceType index>
	Edge& GetPiece(Piece<index>) {
		return *(edges[static_cast<size_t>(index)]);
	}
	template<PieceType index>
	const Edge& GetPiece(Piece<index>) const {
		return *(edges[static_cast<size_t>(index)]);
	}

	CurveRelativePositionType RelativePositionType() const {
		return positionType;
	}

	std::optional<Edge> edges[4];
	CurveRelativePositionType positionType;
};

template <EdgeConcept Edge>
struct SplitEdgeResult {
	template<PieceType index>
	bool HasPiece(Piece<index>) const {
		return edges[static_cast<size_t>(index)].has_value();
	}

	template<PieceType index>
	Edge& GetPiece(Piece<index>) {
		return *(edges[static_cast<size_t>(index)]);
	}
	std::optional<Edge> edges[2];
};

template<class Iterator, class Edge>
concept EdgeIteratorConcept = requires(Iterator a, Iterator b) {
	++a;
	a != b;
	a = b;
	*a; requires std::same_as<std::remove_cvref_t<decltype(*a)>, Edge>;
};

template<class EdgeRelativePositionResult, class Edge>
concept EdgeRelativePositionConcept = requires(EdgeRelativePositionResult result) {
	result.HasPiece(AI); requires std::is_convertible_v<decltype(result.HasPiece(AI)), bool>;
	result.HasPiece(IB); requires std::is_convertible_v<decltype(result.HasPiece(IB)), bool>;
	result.HasPiece(CI); requires std::is_convertible_v<decltype(result.HasPiece(CI)), bool>;
	result.HasPiece(ID); requires std::is_convertible_v<decltype(result.HasPiece(ID)), bool>;

	result.GetPiece(AI); requires std::same_as<std::remove_cvref_t<decltype(result.GetPiece(AI))>, Edge>;
	result.GetPiece(IB); requires std::same_as<std::remove_cvref_t<decltype(result.GetPiece(IB))>, Edge>;
	result.GetPiece(CI); requires std::same_as<std::remove_cvref_t<decltype(result.GetPiece(CI))>, Edge>;
	result.GetPiece(ID); requires std::same_as<std::remove_cvref_t<decltype(result.GetPiece(ID))>, Edge>;

	{ result.RelativePositionType() }->std::same_as<CurveRelativePositionType>;
};

template<class SplitEdgeResult, class Edge>
concept SplitEdgeResultConcept = requires(SplitEdgeResult result, Edge a) {
	result.HasPiece(AI); requires std::is_convertible_v<decltype(result.HasPiece(AI)), bool>;
	result.HasPiece(IB); requires std::is_convertible_v<decltype(result.HasPiece(IB)), bool>;

	result.GetPiece(AI); requires std::same_as<std::remove_cvref_t<decltype(result.GetPiece(AI))>, Edge>;
	result.GetPiece(IB); requires std::same_as<std::remove_cvref_t<decltype(result.GetPiece(IB))>, Edge>;
};

/**
 * @brief 边分析器概念
 */
template<class EdgeAnalyzer, class Edge>
concept EdgeAnalyzerConcept = requires(EdgeAnalyzer ea, Edge a, Edge b) {
	ea.Start(a);
	ea.End(a);

		requires std::same_as<
			decltype(ea.Start(a)), decltype(ea.End(a))
		>;

	ea.Reverse(a); requires std::same_as<Edge, std::remove_cvref_t<decltype(ea.Reverse(a))>>;

	ea.SplitToMonotonic(a); requires std::same_as<MonotonicSplitResult<Edge>, std::remove_cvref_t<decltype(ea.SplitToMonotonic(a))>>;

	// 重命名? classify edge relative elevation
	//ea.CalcateEdgeRelativePosition(a, b);
	//	requires EdgeRelativePositionConcept<decltype(ea.CalcateEdgeRelativePosition(a, b)), Edge>;
	//	requires VertexConcept<std::remove_cvref_t<decltype(ea.Start(a))>>;

	// vertex 还是 point ?
	// 重命名? classify vertex relative position/bearing
	{ ea.CalcateVertexRelativePosition(ea.Start(a), ea.End(a)) }->std::same_as<VertexRelativePositionType>;
	{ ea.CalcateVertexRelativePosition(ea.End(a), ea.Start(a)) }->std::same_as<VertexRelativePositionType>;
	{ ea.CalcateVertexRelativePosition(ea.Start(a), ea.Start(a)) }->std::same_as<VertexRelativePositionType>;
	{ ea.CalcateVertexRelativePosition(ea.End(a), ea.End(a)) }->std::same_as<VertexRelativePositionType>;

	// 计算点是否在边上
	(bool)ea.IsOnEdge(ea.Start(a), a);

	// HasPiece(AI) GetPiece(AI)
	// HasPiece(IB) GetPiece(IB)
	ea.SplitEdge(a, ea.Start(a));
		requires SplitEdgeResultConcept<decltype(ea.SplitEdge(a, ea.Start(a))), Edge>;
};

template<class EdgeAnalyzer, class Edge>
struct edge_analysis_vertex {
	using Vertex = std::remove_cvref_t<decltype(std::declval<EdgeAnalyzer>().Start(std::declval<Edge>()))>;
	using type = Vertex;
};

template<class EdgeAnalyzer, class Edge>
using edge_analysis_vertex_t = edge_analysis_vertex<EdgeAnalyzer, Edge>::type;

class TailorError :public std::runtime_error {
public:
	template<class... Args>
	TailorError(Args&&... args) :std::runtime_error(std::forward<Args>(args)...) {
	}
};

template<class Condition>
class Deathrattle {
private:
	Condition condition;

public:
	Deathrattle() = default;
	Deathrattle(Condition&& condition) :condition(std::move(condition)) {
	}
	Deathrattle(const Condition& condition) :condition(condition) {
	}
	Deathrattle(const Deathrattle&) = delete;
	Deathrattle(Deathrattle&&) noexcept = delete;
	Deathrattle& operator=(const Deathrattle&) = delete;
	Deathrattle& operator=(Deathrattle&&) noexcept = delete;

	~Deathrattle() {
		condition();
	}
};
TAILOR_NAMESPACE_END