package frc.robot.motion.generation.rmpflow;

import org.ejml.simple.SimpleMatrix;

/**
 * A leaf node with no children that contains an RMP.
 */
public abstract class RMPLeaf extends RMPNode{
	/**
	 * RMP leaf node containing an RMP.
	 * @param name of leaf node
	 * @param parent of leaf node
	 */
	public RMPLeaf(String name, RMPNode parent)
	{
		super(name, parent);
	}
	
	@Override
	public final void pullback()
	{
		evaluate();
	}

	/**
	 * Solves for the M and F of the RMP.
	 */
	public final void evaluate()
	{	
		setM(solveM(getX(), getXdot()));
		setF(solveF(getX(), getXdot()));
	}
	
	/**
	 * Creates a square matrix with diagonals of 1s from top left to bottom right.
	 * @param size the numbers of rows for a square matrix
	 * @return A diagonal matrix of 1s.
	 */
	public static SimpleMatrix eye(int size)
	{
		SimpleMatrix eye = new SimpleMatrix(size, size);
		for(int i = 0; i < size; i++)
		{
			eye.set(i , i, 1);
		}
		return eye;
	}

	/**
	 * Solves for F
	 * , where F is the motion policy that describes the dynamical system as a second-order differential equation that uses position and velocity.
	 * @return F 
	 */
	protected abstract SimpleMatrix solveF(SimpleMatrix x, SimpleMatrix x_dot);
	
	/**
	 * Solves for M
	 * , where M is the canonical version of the Riemannian metric A
	 * @return M
	 */
	protected abstract SimpleMatrix solveM(SimpleMatrix x, SimpleMatrix x_dot);
}
