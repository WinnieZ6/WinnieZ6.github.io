# Interior Point Method (IPM)
The **interior point method** (IPM) is a powerful optimization algorithm used primarily to solve convex optimization problems. 

To understand it, we will go over **duality**, **Karush-Kuhn-Tucker (KKT) conditions**, and how they lead to the development of interior point methods.

---
## 1. **Duality in Optimization**
In optimization, the **primal problem** is generally formulated as:

$$
\text{minimize } f(x)
$$
subject to:
$$
g_i(x) \leq 0, \quad i = 1, \dots, m
$$
$$
h_j(x) = 0, \quad j = 1, \dots, p
$$

Where $f(x)$ is the objective function, $g_i(x)$ are inequality constraints, and $h_j(x)$ are equality constraints.

The **dual problem** provides a lower bound to the primal problem’s objective. The dual problem is derived from the Lagrangian function of the primal problem:

$$
L(x, \lambda, \nu) = f(x) + \sum_{i=1}^m \lambda_i g_i(x) + \sum_{j=1}^p \nu_j h_j(x)
$$

Where $\lambda_i \geq 0$ are the Lagrange multipliers for inequality constraints, and $\nu_j$ are the Lagrange multipliers for equality constraints.

To get dual function, we *partial differentiate* the variables and get a equation describing their relationship relative to $\lambda_i$ and $v_j$. 

The **dual function** is:

$$
d(\lambda, \nu) = \inf_x L(x, \lambda, \nu)
$$

The dual problem is:

$$
\text{maximize } d(\lambda, \nu)
$$
subject to:
$$
\lambda_i \geq 0, \quad i = 1, \dots, m
$$

The optimal value of the dual problem provides a bound on the primal problem's optimal value.

Note:
The **dual function** $d(\lambda)$ represents a **lower bound** on the optimal value of the primal problem. By maximizing the dual function, we aim to get the **best lower bound** (which, under certain conditions, will be equal to the optimal value of the primal problem).

In simple terms:
- **Maximizing** the dual function helps us improve our estimate of the primal objective.
- When we maximize the dual function, we can either get the exact value of the primal problem's optimal solution (if **strong duality** holds), or at least a good approximation of it.

### Proof
## Lower bounds on optimal value

The dual function yields lower bounds on the optimal value $ p^* $ of the problem (5.1):  
For any $ \lambda \geq 0 $ and any $ \nu $ we have

$$g(\lambda, \nu) \leq p^*$$  

This important property is easily verified. Suppose $ \bar{x} $ is a feasible point for the problem (5.1), i.e., $ f_i(\bar{x}) \leq 0 $ and $ h_i(\bar{x}) = 0 $, and $ \lambda \geq 0 $. Then we have

$$ \sum_{i=1}^{m} \lambda_i f_i(\bar{x}) + \sum_{i=1}^{p} \nu_i h_i(\bar{x}) \leq 0, $$  
since each term in the first sum is nonpositive, and each term in the second sum is zero, and therefore

$$ L(\bar{x}, \lambda, \nu) = f_0(\bar{x}) + \sum_{i=1}^{m} \lambda_i f_i(\bar{x}) + \sum_{i=1}^{p} \nu_i h_i(\bar{x}) \leq f_0(\bar{x}). $$

Hence

$$g(\lambda, \nu) = \inf_{x \in \mathcal{D}} L(x, \lambda, \nu) \leq L(\bar{x}, \lambda, \nu) \leq f_0(\bar{x}).$$

Since $ g(\lambda, \nu) \leq f_0(\bar{x}) $ holds for every feasible point $ \bar{x} $, the inequality (5.2) follows. The lower bound (5.2) is illustrated in figure 5.1, for a simple problem with $ x \in \mathbb{R} $ and one inequality constraint.

The inequality (5.2) holds, but is vacuous, when $ g(\lambda, \nu) = -\infty $. The dual function gives a nontrivial lower bound on $ p^* $ only when $ \lambda \geq 0 $ and $ (\lambda, \nu) \in \text{dom} \, g $, i.e., $ g(\lambda, \nu) > -\infty $. We refer to a pair $ (\lambda, \nu) $ with $ \lambda \geq 0 $ and $ (\lambda, \nu) \in \text{dom} \, g $ as dual feasible, for reasons that will become clear later.


---
## 2. **Karush-Kuhn-Tucker (KKT) Conditions**
The **KKT conditions** are necessary conditions for a solution to be optimal in constrained optimization problems, assuming certain regularity conditions hold. These conditions are derived from the Lagrangian function and are crucial in both primal and dual optimization.

The **KKT conditions** for a problem with inequality constraints are:

- **Primal feasibility**: 
  $$
  g_i(x) \leq 0, \quad \forall i
  $$

- **Dual feasibility**: 
  $$
  \lambda_i \geq 0, \quad \forall i
  $$

- **Complementary slackness**:
  $$
  \lambda_i g_i(x) = 0, \quad \forall i
  $$

- **Stationarity**:
  $$
  \nabla_x L(x, \lambda, \nu) = 0
  $$

Where $L(x, \lambda, \nu)$ is the Lagrangian, and $\lambda_i$ are the Lagrange multipliers.
![[CleanShot 2025-06-11 at 15.48.15@2x.png]]

$$x_{t+1} = -\frac{f^1(x)}{f(x)}+x_{t}$$

---

#### 3. **Interior Point Method (IPM)**
The **interior point method** is an iterative algorithm that approaches the solution of the optimization problem from within the feasible region, as opposed to boundary-based methods like the simplex method. 

The general idea is to solve the **KKT conditions** by maintaining strictly positive values for the slack variables (those associated with inequality constraints) at every iteration. The algorithm works by solving a series of **perturbed KKT conditions**.

The **perturbed KKT conditions** for a primal problem are:

- **Primal feasibility**: 
  $$
  g_i(x) \leq 0, \quad \forall i
  $$

- **Dual feasibility**: 
  $$
  \lambda_i \geq 0, \quad \forall i
  $$

- **Complementary slackness**:
  $$
  \lambda_i g_i(x) = \sigma, \quad \forall i
  $$

Where $\sigma$ is a small positive perturbation value, typically chosen to ensure that $\lambda_i$ and $g_i(x)$ do not become zero, allowing the algorithm to stay in the interior of the feasible region.

$$-\sum^{m}_{i=1}\log(-g_{i}(x)) $$
the function will approach $\infty$ when $g_{i}(x)$ approaches to 0.
Thus, the overall objective becomes:
$$\min_{x} f(x) - \mu \sum^{m}_{i=1}\log(-g_{i}(x))$$
Here, $\mu$ is a small positive parameter that controls the barrier strength. As $\mu$ decreases, the barrier function has less influence, and the optimization process moves closer to the boundary (near the constraints).

The IPM works by iteratively solving a **Newton system** that approximates the behavior of the perturbed KKT system:

$$
\begin{bmatrix}
H(x_k) & A^T \\
A & 0
\end{bmatrix}
\begin{bmatrix}
\Delta x_k \\
\Delta \lambda_k
\end{bmatrix}
=
-\begin{bmatrix}
\nabla f(x_k) + A^T \lambda_k \\
b - Ax_k
\end{bmatrix}
$$

Where:
- $H(x_k)$ is the Hessian of the Lagrangian (or approximation of it),
- $A$ is the constraint matrix,
- $\Delta x_k$ and $\Delta \lambda_k$ are the search directions for the primal and dual variables at iteration $k$.

At each step, the algorithm updates the primal and dual variables by solving this system, gradually approaching the optimal solution while staying in the interior of the feasible region.

---

### Conclusion
The interior point method uses the duality theory and KKT conditions to solve optimization problems efficiently. It iterates toward the optimal solution while maintaining strictly positive slack variables, which ensures the algorithm remains within the interior of the feasible region. This method has been particularly successful in solving large-scale linear programming problems and convex optimization problems.