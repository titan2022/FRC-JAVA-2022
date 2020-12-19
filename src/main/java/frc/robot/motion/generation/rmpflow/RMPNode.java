package frc.robot.motion.generation.rmpflow;
import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

/**
 * Represents a single node and its connections in an Riemannian Motion Policies Tree.
 * Includes implementation as described in the RMPFlow algorithm.
 */
public abstract class RMPNode {
	private String name;
	private RMPNode parent;
	private ArrayList<RMPNode> children = new ArrayList<RMPNode>();
	private SimpleMatrix x, x_dot, f, m;
	
	/**
	 * A node for an tree that contains mappings and functions for Riemannian Motion Policies.
	 * @param name of RMP node
	 * @param parent of RMP node
	 */
	public RMPNode(String name, RMPNode parent)
	{
		this.name = name;
		this.parent = parent;
		if(parent != null)
			parent.addChild(this);
		this.x = null;
		this.x_dot = null;
		this.f = null;
		this.m = null;
	}

	/**
	 * Differentiable task map that relates the configuration space to the task space.
	 * @param q is the configuration space
	 * @return the task space
	 */
	public SimpleMatrix phi(SimpleMatrix q)
	{
		return q;
	}

	/**
	 * Jacobian of the task map phi
	 * @param q is the configuration space
	 * @return
	 */
	public SimpleMatrix j(SimpleMatrix q)
	{
		return q;
	};

	/**
	 * Second derivative Jacobian of the task map phi
	 * @param q is the configuration space
	 * @param q_dot is the configuration space
	 * @return differentiated Jacobian of configuration space to task space 
	 */
	public SimpleMatrix j_dot(SimpleMatrix q, SimpleMatrix q_dot)
	{
		return q_dot;//TODO decide if to implement like goal Attractor J_dot
	};
	
	/**
	 * Adds a node as a child to the implicit node.
	 * @param child A RMP node child
	 */
	public void addChild(RMPNode child)
	{
		children.add(child);
	}
	
	/**
	 * Returns all child nodes.
	 * @return An array list of all children nodes.
	 */
	public ArrayList<RMPNode> getChildren()
	{
		return children;
	}
	
	/**
	 * 	The operator to forward propagate the state from a parent node to its child nodes.
	 *  I.e. Transform an RMP from the domain of a task map to its co-domain.
	 * 	Equation 12 in Riemannian Motion Policies.
	 */
	public void pushforward()
	{
		//In the case that an RMP node does not have a psi or j function implement psi and j to return the input.
		//TODO: Figure out above case
		x = phi(parent.x);//psi(x)
		x_dot = j(parent.x).mult(parent.x_dot);//j(x) * x_dot
		
		for(int i = 0; i < children.size(); i++)
			children.get(i).pushforward();
	}
	/**
	 * The operator to backward propagate the natural-formed RMPs from the child nodes to the parent node.
	 * I.e update parent states
	 * Equation 11 in Riemannian Motion Policies
	 */
	public void pullback()
	{
		for(int i = 0; i < children.size(); i++)
			children.get(i).pullback();
		
		f = x.copy();
		f.zero();

		int mSize = Math.max(x.numCols(), x.numRows());
		if(m == null)
			m = new SimpleMatrix(mSize, mSize);
		else
			m.reshape(mSize, mSize);
		m.zero();
		
		for(int i = 0; i < children.size(); i++)
		{
			RMPNode child = children.get(i);
			SimpleMatrix J_child = child.j(x);//child's Jacobian of x
			SimpleMatrix J_dot_child = child.j_dot(x, x_dot);
			
			if(child.f != null && child.name != null)
			{
				//f + JT * (f - (m * J_dot * x_dot)) Equation 1 in RMPFlow Computational Graph
				f = f.plus(J_child.transpose().mult(child.f.minus(
								child.m.mult(J_dot_child).mult(x_dot))));
				//JT * M * J Equation 1 in RMPFlow Computational Graph
				m = m.plus(J_child.transpose().mult(child.m).mult(J_child));
			}
		}
	}

	/**
	 * Returns current state
	 * @return x
	 */
	public SimpleMatrix getX() {return x;}

	/**
	 * Sets the current state
	 * @param x
	 */
	public void setX(SimpleMatrix x) {this.x = x;}

	/**
	 * Gets the current derivative of the state
	 * @return first order of system
	 */
	public SimpleMatrix getXdot() {return x_dot;}

	/**
	 * Sets the current first order state
	 * @param xdot is the first order of the system
	 */
	public void setXdot(SimpleMatrix xdot) {this.x_dot = xdot;}

	/**
	 * Returns acceleration motion policy
	 * @return f
	 */
	public SimpleMatrix getF() {return f;}

	/**
	 * Sets acceleration motion policy
	 * @param f
	 */
	public void setF(SimpleMatrix f) {this.f = f;}

	/**
	 * Returns riemannian metric
	 * @return M
	 */
	public SimpleMatrix getM() {return m;}

	/**
	 * Sets riemannian metric
	 * @param m
	 */
	public void setM(SimpleMatrix m) {this.m = m;}

	/**
	 * Computes and returns motion policy as a = M†f.
	 * † denotes Moore-Penrose inverse.
	 * @return A
	 */
	public SimpleMatrix getA() {return m.pseudoInverse().mult(f);}
}