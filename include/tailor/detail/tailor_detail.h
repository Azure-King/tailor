#pragma once

#include "tailor.h"

template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer>
template<class VEQ>
inline void Tailor<Edge, EdgeAnalyzer>::EdgeStateSet<VEQ>::
AcceptSameLocEndEvents(std::vector<VEvent>& end_events, TopoVertex& vertex) {
	for (auto begin = events.begin(); begin != events.end(); ++begin) {
		if (end_events.empty()) break;

		auto end = begin; // 需要移除的区间末尾
		auto event_end = events.end();
		for (; !end_events.empty() && end != events.end(); ++end) {
			auto remove_it = std::remove_if(end_events.begin(), end_events.end(),
				[eid = end->e](const VEvent& ve) {return ve.e == eid; }
			);
			if (end_events.end() == remove_it) break;
			end_events.erase(remove_it, end_events.end());
		}

		// 没有需要要删除的事件
		if (begin == end) continue;

		// 结束连续的事件
		std::for_each(begin, end, [this, &vertex](const VEvent& ve) {
			EdgeEvent& ee = GetEdgeEvent(ve.e);
			ee.endPntGroup = vertex.id;
			EndEvent(ee); // 结束被移除的事件
			});

		// 删除 [begin, end)
		begin = events.erase(begin, end);

		if (events.end() == begin /* 上方无边 */) break;
		if (events.begin() == begin /* 下方无边 */) continue;

		//VertexRelativePositionType vrp = ea.CalcateVertexRelativePosition((begin - 1)->v, begin->v);
		const VertexRelativePositionType vrp = ea.CalcateVertexRelativePosition(
			GetPointInVertexEvent(*(begin - 1)), GetPointInVertexEvent(*begin)
		);

		const bool reversed = Negative(vrp);

		// 由于比较的两个事件可能不满足 A <= C, 因此需要判断
		const auto ab_it = reversed ? begin : (begin - 1);
		const auto cd_it = reversed ? (begin - 1) : begin;

		// 移除事件后, 将上下的边事件进行求交处理
		auto erp = ea.CalcateEdgeRelativePosition(
			GetEdgeEvent(ab_it->e).edge,
			GetEdgeEvent(cd_it->e).edge,
			OnlyFocusOnRelativePositionDetailsWithoutCoincidence{}
		);

		//const auto& debug_edge0 = GetEdgeEvent(ab_it->e);
		//const auto& debug_edge1 = GetEdgeEvent(cd_it->e);
		assert(erp.HasPiece(CI));

		VEventGroup group{};
		if (erp.HasPiece(IB)) {
			auto spr = SplitEvent(ab_it->e, erp.GetPiece(AI), erp.GetPiece(IB));

			// 将 ab_it 指向的起始点事件替换为 AI 的起始点事件
			*ab_it = MakeStartVertexEvent(spr.aiEvent.id);

			group.Add(
				spr.aiEvent.id, false,
				spr.ibEvent.id, true
			);

			// 重新加入事件: AI 的终点事件、IB 的起点事件、IB 的终点事件
			//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
			//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
			//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
		}

		if (erp.HasPiece(ID)) {
			auto spr = SplitEvent(cd_it->e, erp.GetPiece(CI), erp.GetPiece(ID));

			// 将 cd_it 指向的起始点事件替换为 CI 的起始点事件
			*cd_it = MakeStartVertexEvent(spr.aiEvent.id);

			group.Add(
				spr.aiEvent.id, false,
				spr.ibEvent.id, true
			);

			// 重新加入事件: CI 的终点事件、ID 的起点事件、ID 的终点事件
			//veq.Push(MakeEndVertexEvent(spr.aiEvent.id));
			//veq.Push(MakeStartVertexEvent(spr.ibEvent.id));
			//veq.Push(MakeEndVertexEvent(spr.ibEvent.id));
		}
		if (group.edges[0] != npos) {
			veq.Push(group);
		}
	}
}

template<EdgeConcept Edge, EdgeAnalyzerConcept<Edge> EdgeAnalyzer>
inline auto Tailor<Edge, EdgeAnalyzer>::Execute() -> PatternDrafting {
	std::vector<VEventGroup> container;
	std::vector<EdgeEvent> edge_events;

	//container.reserve(20000);
	container.reserve(static_cast<size_t>((clipper.size() + subject.size()) * 2.5));
	VertexEventQueue<> queue(ea, edge_events, std::move(container));
	EdgeStateSet<decltype(queue)> set(queue, edge_events, ea, clipper, subject);

	// 顶点事件入队 

	for (size_t i = 0, n = set.edgeEvents.size(); i < n; ++i) {
		auto& edge = set.edgeEvents[i];
		if (edge.discarded) continue;
		queue.Push(i, true);
		queue.Push(i, false);
		//queue.Push(ea.Start(edge.edge), i, true);
		//queue.Push(ea.End(edge.edge), i, false);
	}

	std::vector<VEvent> start_events;
	std::vector<VEvent> end_events;

	//size_t times = 0;
	// 顶点事件出队
	while (!queue.Empty()) {
		//times++;

		//if (times == 130) {
		//	int ccc = 0;
		//}

		// 取出所有的相同位置的顶点事件
		queue.PopAllEvents(start_events, end_events);
		// 现在应该可以禁用 RemoveDiscardedEvent 了
		//set.RemoveDiscardedEvent(start_events);
		//set.RemoveDiscardedEvent(end_events);

		// 事件处理的有效边总数必须为偶数
		assert((
			set.CalcVaildEdgeSize(start_events) +
			set.CalcVaildEdgeSize(end_events))
			% 2 == 0);

		//for (auto& se : start_events)
		//{
		//	const auto& edge = set.GetEdgeEvent(se.e);
		//	this->DebugFunc3(edge, se.start);
		//}

		//for (auto& ee : end_events)
		//{
		//	const auto& edge = set.GetEdgeEvent(ee.e);
		//	this->DebugFunc3(edge, ee.start);
		//}

		auto& vertex = set.RegisterVertex();

		set.PreprocessEvents(start_events, end_events);

		set.AcceptSameLocEndEvents(end_events, vertex);

		set.AcceptSameLocStartEvents(start_events, vertex);

		end_events.clear();
		start_events.clear();
	}
	return PatternDrafting{ std::move(edge_events),std::move(set.topoVertices) };
}
