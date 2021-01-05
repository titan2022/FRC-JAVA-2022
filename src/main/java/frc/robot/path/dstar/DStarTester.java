package frc.robot.path.dstar;

import java.util.PriorityQueue;
import java.util.Queue;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mapping.LinearSegment;
import frc.robot.mapping.Point;

public class DStarTester extends CommandBase {
    @Override
    public void initialize() {
        System.out.print("Node methods: ");
        System.out.println(testNode());
        System.out.print("D* Lite primitives: ");
        System.out.println(testPrimitives());
    }

    @Override
    public void execute() {
        return;
    }

    public boolean testNode() {
        boolean truth = true;
        boolean allPass = true;
        Queue<DStarNode> queue = new PriorityQueue<>();
        DStarNode a = new DStarNode(new Point(0, 0), queue);
        truth = a.getDegree() == 0;
        allPass &= truth;
        truth = a.key() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        truth = a.isConsistent();
        allPass &= truth;
        DStarNode b = new DStarNode(new Point(1, 0), queue, 0, 0);
        truth = b.key() == 0;
        allPass &= truth;
        truth = b.isConsistent();
        allPass &= truth;
        a.connect(b, new LinearSegment(a, b));
        b.connect(a, new LinearSegment(b, a));
        truth = a.getDegree() == 1;
        allPass &= truth;
        truth = b.getDegree() == 1;
        allPass &= truth;
        truth = a.getG() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        truth = b.getG() == 0;
        allPass &= truth;
        truth = a.getRhs() == 1;
        allPass &= truth;
        truth = b.getRhs() == 0;
        allPass &= truth;
        truth = a.getNext() == b;
        allPass &= truth;
        truth = b.getNext() == null;
        allPass &= truth;
        truth = queue.contains(a);
        allPass &= truth;
        truth = !queue.contains(b);
        allPass &= truth;

        DStarNode c = new DStarNode(new Point(0, 2), queue, 5, Double.POSITIVE_INFINITY);
        truth = !c.isConsistent();
        allPass &= truth;
        a.connect(c, new LinearSegment(a, c));
        c.connect(a, new LinearSegment(c, a));
        truth = a.getDegree() == 2;
        allPass &= truth;
        truth = c.getDegree() == 1;
        allPass &= truth;
        truth = c.getRhs() == a.getG() + 2;
        allPass &= truth;
        truth = c.getRhs() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        a.sever(b);
        truth = a.getDegree() == 1;
        allPass &= truth;
        truth = b.getDegree() == 0;
        allPass &= truth;
        truth = a.getRhs() == c.getG() + 2;
        allPass &= truth;
        return allPass;
    }

    public boolean testPrimitives() {
        boolean truth = true;
        boolean allPass = true;
        Queue<DStarNode> queue = new PriorityQueue<>();
        DStarNode a = new DStarNode(new Point(1, 1), queue);
        a.update();
        truth = queue.size() == 0;
        allPass &= truth;
        a.rectify();
        truth = queue.size() == 0;
        allPass &= truth;
        truth = a.getG() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        truth = a.getRhs() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        DStarNode b = new DStarNode(new Point(0, 1), queue, 0, 1);
        b.update();
        truth = queue.size() == 1;
        allPass &= truth;
        b.rectify();
        truth = queue.size() == 0;
        allPass &= truth;
        truth = b.getG() == Double.POSITIVE_INFINITY;
        allPass &= truth;
        truth = b.getRhs() == 1;
        allPass &= truth;
        DStarNode c = new DStarNode(new Point(1, 0), queue, 1, 0);
        c.update();
        truth = queue.size() == 1;
        allPass &= truth;
        c.rectify();
        truth = queue.size() == 0;
        allPass &= truth;
        truth = c.getG() == 0;
        allPass &= truth;
        truth = c.getRhs() == 0;
        allPass &= truth;
        DStarNode d = new DStarNode(new Point(0, 0), queue, 0, 0);
        d.update();
        truth = queue.size() == 0;
        allPass &= truth;
        d.rectify();
        truth = queue.size() == 0;
        allPass &= truth;
        truth = d.getG() == 0;
        allPass &= truth;
        truth = d.getRhs() == 0;
        allPass &= truth;
        return allPass;
    }
}