package frc.robot.path.dstar;

import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.NavigableMap;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.mapping.LinearSegment;
import frc.robot.mapping.Obstacle;
import frc.robot.mapping.ObstacleMap;
import frc.robot.mapping.Path;
import frc.robot.mapping.Point;

public class DStarGraph {
    private final Queue<DStarNode> queue = new PriorityQueue<DStarNode>();
    private final DStarNode goal;
    private DStarNode start;
    public final ObstacleMap map;
    public final double radius;
    private Map<Obstacle, NavigableMap<Point, DStarNode>> obstacleSets = new HashMap<>();

    public DStarGraph(ObstacleMap map, Translation2d start, Translation2d goal, double radius) {
        this.map = map;
        this.goal = new DStarNode(goal, queue, 0, 0);
        this.start = new DStarNode(start, queue);
        this.radius = radius;
    }
    public DStarGraph(ObstacleMap map, Translation2d start, Translation2d goal) {
        this(map, start, goal, 0);
    }
    public DStarGraph(ObstacleMap map, Translation2d goal, double radius) {
        this(map, goal, goal, radius);
    }
    public DStarGraph(ObstacleMap map, Translation2d goal) {
        this(map, goal, 0);
    }

    public Iterable<DStarNode> getNodes() {
        Set<DStarNode> res = new LinkedHashSet<>();
        res.add(goal);
        res.add(start);
        for(Map<Point, DStarNode> map : obstacleSets.values())
            res.addAll(map.values());
        return res;
    }

    public Iterable<Path> getEdges() {
        return new Iterable<Path>() {
            @Override
            public Iterator<Path> iterator() {
                return new Iterator<Path>() {
                    Iterator<DStarNode> nodeIter = getNodes().iterator();
                    Iterator<Path> edgeIter = null;
                    @Override
                    public boolean hasNext() {
                        return nodeIter.hasNext() || (edgeIter != null && edgeIter.hasNext());
                    }
                    @Override
                    public Path next() {
                        while(edgeIter == null || !edgeIter.hasNext())
                            edgeIter = nodeIter.next().getEdges().iterator();
                        return edgeIter.next();
                    }
                };
            }
        };
    }

    public boolean addNode(Point position, Obstacle obstacle) {
        NavigableMap<Point, DStarNode> obsSet = obstacleSets.get(obstacle);
        if(obsSet.putIfAbsent(position, new DStarNode(position, queue)) != null)
            return false;
        if(obsSet.size() > 1){
            DStarNode node = obsSet.get(position);
            Map.Entry<Point, DStarNode> prevEntry = obsSet.floorEntry(position);
            DStarNode prev = (prevEntry == null ? obsSet.lastEntry() : prevEntry).getValue();
            Map.Entry<Point, DStarNode> nextEntry = obsSet.ceilingEntry(position);
            DStarNode next = (nextEntry == null ? obsSet.firstEntry() : nextEntry).getValue();
            prev.sever(next);
            next.sever(prev);
            Path prevEdge = obstacle.edgePath(prev, node, radius);
            prev.connect(node, prevEdge);
            node.connect(prev, prevEdge.reverse());
            Path nextEdge = obstacle.edgePath(node, next, radius);
            node.connect(next, nextEdge);
            next.connect(node, nextEdge.reverse());
        }
        return true;
    }

    public void dropNode(Point position, Obstacle obstacle) {
        NavigableMap<Point, DStarNode> obsSet = obstacleSets.get(obstacle);
        obsSet.remove(position);
        DStarNode prev, next;
        if(obsSet.size() > 0){
            DStarNode node = obsSet.get(position);
            Map.Entry<Point, DStarNode> prevEntry = obsSet.floorEntry(position);
            prev = (prevEntry == null ? obsSet.lastEntry() : prevEntry).getValue();
            Map.Entry<Point, DStarNode> nextEntry = obsSet.ceilingEntry(position);
            next = (nextEntry == null ? obsSet.firstEntry() : nextEntry).getValue();
            node.sever(next);
            node.sever(prev);
            if(obsSet.size() > 1){
                Path edge = obstacle.edgePath(prev, next, radius);
                prev.connect(next, edge);
                next.connect(prev, edge.reverse());
            }
        }
    }
    public void dropNode(Point position) {
        dropNode(position, findNode(position));
    }

    Obstacle findNode(Point position) {
        for(Map.Entry<Obstacle, NavigableMap<Point, DStarNode>> entry : obstacleSets.entrySet())
            if(entry.getValue().containsKey(position))
                return entry.getKey();
        return null;
    }

    void addEdge(Obstacle a, Obstacle b, Path edge) {
        addNode(edge.getStart(), a);
        addNode(edge.getEnd(), b);
        DStarNode beg = obstacleSets.get(a).get(edge.getStart());
        DStarNode end = obstacleSets.get(b).get(edge.getEnd());
        beg.connect(end, edge);
        end.connect(beg, edge.reverse());
    }
}
