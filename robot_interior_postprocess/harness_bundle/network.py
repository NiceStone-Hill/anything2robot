"""Geometry-independent packing and cable membership on a shared tree."""

import itertools
from collections import defaultdict, deque
from functools import lru_cache

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra


@lru_cache(maxsize=256)
def pack(diameters, gap=0.2, sleeve=0.5):
    """Explicit, non-overlapping circular cable cross sections; millimetres.

    This reserves room for insulated cable envelopes, not bare conductors.
    Layout transport through bends and branch fan-out remains a separate task.
    """
    if not diameters or not np.isfinite(diameters).all() or min(diameters) <= 0:
        raise ValueError("Cable diameters must be finite and positive")
    if not np.isfinite([gap, sleeve]).all() or min(gap, sleeve) < 0:
        raise ValueError("Invalid packing allowance")
    placed = []
    angles = np.linspace(0, 2 * np.pi, 96, endpoint=False)
    directions = np.column_stack([np.cos(angles), np.sin(angles)])
    for diameter in sorted(diameters, reverse=True):
        r = diameter / 2
        if not placed:
            placed.append((0.0, 0.0, r))
            continue
        old = np.asarray(placed)
        candidates = (
            old[:, None, :2] + directions[None] * (old[:, None, 2:3] + r + gap)
        ).reshape(-1, 2)
        ok = np.all(
            np.linalg.norm(candidates[:, None] - old[None, :, :2], axis=2)
            >= old[None, :, 2] + r + gap - 1e-8,
            axis=1,
        )
        candidates = candidates[ok]
        outer = np.max(np.linalg.norm(old[:, :2], axis=1) + old[:, 2])
        score = np.maximum(
            outer, np.linalg.norm(candidates, axis=1) + r
        ) + 1e-5 * np.linalg.norm(candidates, axis=1)
        p = candidates[np.argmin(score)]
        placed.append((float(p[0]), float(p[1]), r))
    radius = max(np.hypot(x, y) + r for x, y, r in placed) + sleeve
    return float(radius), tuple(placed)


def rooted_grid_tree(free, clearance, root):
    """One shortest-path tree shared by all devices, favouring open corridors."""
    cells = np.argwhere(free)
    ids = np.full(free.shape, -1, np.int32)
    ids[tuple(cells.T)] = np.arange(len(cells))
    rows, cols, weights = [], [], []
    for axis in range(3):
        a = [slice(None)] * 3
        b = a.copy()
        a[axis] = slice(None, -1)
        b[axis] = slice(1, None)
        a, b = tuple(a), tuple(b)
        both = free[a] & free[b]
        aa, bb = ids[a][both], ids[b][both]
        width = np.minimum(clearance[a][both], clearance[b][both])
        w = 1.0 + (12.0 / np.maximum(width, 1.0)) ** 2
        rows.extend([aa, bb])
        cols.extend([bb, aa])
        weights.extend([w, w])
    graph = coo_matrix(
        (np.concatenate(weights), (np.concatenate(rows), np.concatenate(cols))),
        shape=(len(cells), len(cells)),
    ).tocsr()
    distances, predecessors = dijkstra(
        graph, directed=False, indices=int(ids[tuple(root)]), return_predecessors=True
    )
    return cells, ids, distances, predecessors


def tree_path(a, b, predecessors):
    def ancestors(node):
        result = [int(node)]
        while predecessors[result[-1]] >= 0:
            result.append(int(predecessors[result[-1]]))
        return result

    aa, bb = ancestors(a), ancestors(b)
    positions = {node: i for i, node in enumerate(aa)}
    for j, node in enumerate(bb):
        if node in positions:
            return aa[: positions[node] + 1] + bb[:j][::-1]
    raise ValueError("Disconnected device exits")


def shared_segments(paths, terminal_nodes):
    """Collapse degree-two grid chains; each segment retains exact cable IDs."""
    members = defaultdict(set)
    adjacent = defaultdict(set)
    for cable, path in paths.items():
        for a, b in itertools.pairwise(path):
            key = tuple(sorted((int(a), int(b))))
            members[key].add(cable)
            adjacent[a].add(b)
            adjacent[b].add(a)
    stops = {n for n, v in adjacent.items() if len(v) != 2} | set(terminal_nodes)
    used = set()
    segments = []
    for start in sorted(stops):
        for nxt in sorted(adjacent[start]):
            edge = tuple(sorted((start, nxt)))
            if edge in used:
                continue
            path = [start, nxt]
            used.add(edge)
            cables = members[edge]
            while path[-1] not in stops:
                next_node = next(n for n in adjacent[path[-1]] if n != path[-2])
                edge = tuple(sorted((path[-1], next_node)))
                if members[edge] != cables:
                    raise ValueError(
                        "Membership changed outside a terminal or junction"
                    )
                used.add(edge)
                path.append(next_node)
            segments.append({"nodes": path, "cables": sorted(cables)})
    if len(used) != len(members):
        raise ValueError("Unaccounted tree edges")
    return segments


def cable_segment_path(segments, cable, start, end):
    graph = defaultdict(list)
    for segment in segments:
        if cable in segment["cables"]:
            a, b = segment["nodes"][0], segment["nodes"][-1]
            graph[a].append((b, segment["id"], False))
            graph[b].append((a, segment["id"], True))
    queue = deque([(start, [])])
    seen = {start}
    while queue:
        node, path = queue.popleft()
        if node == end:
            return path
        for nxt, key, reverse in graph[node]:
            if nxt not in seen:
                seen.add(nxt)
                queue.append((nxt, path + [{"segment_id": key, "reverse": reverse}]))
    raise ValueError("Cable has no continuous segment path: " + cable)


def growing_tree(free, clearance, terminal_nodes, start_node):
    """Attach each device to the nearest existing trunk, avoiding parallel trees.

    All nodes already in the harness are zero-distance Dijkstra sources. New
    branches stop at first contact, so the result has no geometric grid cycles.
    """
    cells = np.argwhere(free)
    ids = np.full(free.shape, -1, np.int32)
    ids[tuple(cells.T)] = np.arange(len(cells))
    rows = []
    cols = []
    weights = []
    for axis in range(3):
        a = [slice(None)] * 3
        b = a.copy()
        a[axis] = slice(None, -1)
        b[axis] = slice(1, None)
        a, b = tuple(a), tuple(b)
        both = free[a] & free[b]
        aa, bb = ids[a][both], ids[b][both]
        width = np.minimum(clearance[a][both], clearance[b][both])
        w = 1.0 + (12.0 / np.maximum(width, 1.0)) ** 2
        rows.extend([aa, bb])
        cols.extend([bb, aa])
        weights.extend([w, w])
    graph = coo_matrix(
        (np.concatenate(weights), (np.concatenate(rows), np.concatenate(cols))),
        shape=(len(cells), len(cells)),
    ).tocsr()
    tree = {int(start_node)}
    remaining = {int(n) for n in terminal_nodes} - tree
    parent = np.full(len(cells), -9999, np.int32)
    while remaining:
        dist, pred, _sources = dijkstra(
            graph,
            directed=False,
            indices=sorted(tree),
            min_only=True,
            return_predecessors=True,
        )
        nxt = min(remaining, key=lambda n: (dist[n], n))
        if not np.isfinite(dist[nxt]):
            raise ValueError("Disconnected bundle terminal")
        node = nxt
        while node not in tree:
            following = int(pred[node])
            if following < 0:
                raise ValueError("Unreachable bundle tree")
            parent[node] = following
            tree.add(node)
            node = following
        remaining -= tree
    return parent


def consolidate_junctions(
    paths, terminal_nodes, coordinates, maximum_distance, visible
):
    """Contract short junction chains without merging distinct device exits.

    Close tree junctions otherwise leave sub-diameter intermediate segments and
    overlapping branch envelopes. Each contraction preserves cable identities
    and fixed device exits, and is accepted only inside a visible free region.
    """
    terminals = set(terminal_nodes)
    segments = shared_segments(paths, terminals)
    nodes = {n for path in paths.values() for n in path}
    representative = {n: n for n in nodes}
    clusters = {n: {n} for n in nodes}
    ordered = sorted(
        segments,
        key=lambda s: sum(
            np.linalg.norm(coordinates[b] - coordinates[a])
            for a, b in itertools.pairwise(s["nodes"])
        ),
    )
    for segment in ordered:
        path = segment["nodes"]
        a, b = representative[path[0]], representative[path[-1]]
        if a == b:
            continue
        merging = set().union(*(clusters[representative[n]] for n in path))
        exits = merging & terminals
        if len(exits) > 1:
            continue
        candidates = sorted(exits) if exits else [a, b]
        selected = None
        for center in candidates:
            if any(
                np.linalg.norm(coordinates[n] - coordinates[center]) > maximum_distance
                for n in merging
            ):
                continue
            if not all(visible(coordinates[center], coordinates[n]) for n in merging):
                continue
            selected = center
            break
        if selected is None:
            continue
        for root in {representative[n] for n in merging}:
            clusters.pop(root)
        clusters[selected] = merging
        for n in merging:
            representative[n] = selected
    result = {}
    for cable, path in paths.items():
        mapped = []
        for n in path:
            value = representative[n]
            if not mapped or mapped[-1] != value:
                mapped.append(value)
        if mapped[0] != path[0] or mapped[-1] != path[-1]:
            raise ValueError("Junction merge moved a device exit")
        result[cable] = mapped
    return result
