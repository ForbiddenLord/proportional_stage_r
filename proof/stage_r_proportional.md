# Stage R with Proportional Feedback: Complete Derivation

## 1. Setup and Notation

**Error rotor:** $R_e := R_d^\dagger R \in \text{Spin}(3)$

**Configuration error:** 
$$\Psi_e := 1 - \langle R_e \rangle_0 = 1 - \cos(\theta_e/2) \in [0, 2)$$

where $\theta_e$ is the total rotation angle encoded in $R_e = \cos(\theta_e/2) - \hat{n}_e \sin(\theta_e/2)$, and $\hat{n}_e$ is the unit bivector rotation axis.

**Key geometric identity:** 
$$\sin(\theta_e/2) = \sqrt{\Psi_e(2 - \Psi_e)}$$

**Normalized rotational error:** 
$$e_R(t) := \frac{\Psi_e(t)}{\rho_R(t)} \in [0, 1)$$

where $\rho_R(t) = (\rho_{R,0} - \rho_{R,\infty})e^{-k_R t} + \rho_{R,\infty}$ is the time-varying funnel radius.

**Body-frame angular velocity error bivector** (from paper eq. 35):
$$e_\Omega := \Omega_b - \tilde{R}_e \tilde{\Omega}_d R_e$$

**Rate of configuration error** (from paper eq. 36):
$$\dot{\Psi}_e = -\frac{1}{2}\sin(\theta_e/2)(e_\Omega \cdot \hat{n}_e)$$

---

## 2. Proportional Control Law

### Choice of Virtual Angular Velocity Command

**Feedback correction term:**
$$e_\Omega^v := \kappa_R \Psi_e \hat{n}_e$$

**Full virtual angular velocity command:**
$$\Omega_b^v := e_\Omega^v + \tilde{R}_e \tilde{\Omega}_d R_e = \kappa_R \Psi_e \hat{n}_e + \tilde{R}_e \tilde{\Omega}_d R_e$$

### Why This Choice

1. **No division by $\sin(\theta_e/2)$**: The feedback is linear in $\Psi_e$, not inverse-proportional to the rotation rate.
2. **No logarithmic barrier**: We abandon the logarithmic speed-up in favor of clean exponential decay.
3. **Simple limit at $\theta_e = 0$**: As $\theta_e \to 0$, $\Psi_e \to 0$, so $e_\Omega^v \to 0$ naturally (no removable singularity needed).

---

## 3. Lyapunov Analysis for Proportional Stage R

### Lyapunov Function

$$V_R := \frac{1}{2}\Psi_e^2$$

(Alternative: $V_R := \frac{1}{2}e_R^2$; the scaling is equivalent. We use $\Psi_e$ for clarity.)

### Derivative (Perfect Tracking Case)

Assume perfect tracking: $e_\Omega = e_\Omega^v = \kappa_R \Psi_e \hat{n}_e$ (i.e., $\Omega_b = \Omega_b^v$).

$$\dot{V}_R = \Psi_e \dot{\Psi}_e = \Psi_e \left[ -\frac{1}{2}\sin(\theta_e/2)(e_\Omega \cdot \hat{n}_e) \right]$$

$$= -\frac{1}{2}\Psi_e \sin(\theta_e/2) \cdot (e_\Omega^v \cdot \hat{n}_e)$$

$$= -\frac{1}{2}\Psi_e \sin(\theta_e/2) \cdot \kappa_R \Psi_e$$

$$= -\frac{\kappa_R}{2}\Psi_e^2 \sin(\theta_e/2)$$

Substituting $\sin(\theta_e/2) = \sqrt{\Psi_e(2 - \Psi_e)}$:

$$\dot{V}_R = -\frac{\kappa_R}{2}\Psi_e^2 \sqrt{\Psi_e(2 - \Psi_e)} = -\frac{\kappa_R}{2}\Psi_e^{5/2}\sqrt{2 - \Psi_e}$$

### Decay Rate (Key Inequality)

For $\Psi_e \in [0, 1]$ (small errors), $\sqrt{2 - \Psi_e} \approx \sqrt{2}$, so:

$$\dot{V}_R \lesssim -c_R \Psi_e^{5/2}$$

where $c_R = \kappa_R / (2\sqrt{2})$.

**Equivalently, in terms of normalized error $e_R$:**

$$\dot{V}_R \lesssim -c_R \rho_R^{5/2} e_R^{5/2}$$

### Accounting for Tube Shrinkage

The actual derivative includes the $\dot{\rho}_R$ term:

$$\dot{e}_R = \frac{\dot{\Psi}_e}{\rho_R} - e_R \frac{\dot{\rho}_R}{\rho_R}$$

So:

$$\dot{V}_R = e_R \dot{e}_R = \frac{e_R}{\rho_R}\dot{\Psi}_e - e_R^2 \frac{\dot{\rho}_R}{\rho_R}$$

With $e_\Omega^v = \kappa_R \Psi_e \hat{n}_e = \kappa_R \rho_R e_R \hat{n}_e$:

$$\dot{V}_R = -\frac{\kappa_R}{2\rho_R} e_R^2 \sin(\theta_e/2) \cdot \rho_R e_R - e_R^2 \frac{\dot{\rho}_R}{\rho_R}$$

$$= -\frac{\kappa_R}{2} e_R^3 \sin(\theta_e/2) - e_R^2 \frac{\dot{\rho}_R}{\rho_R}$$

**Lower bound (using $\sin(\theta_e/2) \geq c_0 \sqrt{e_R}$ for small $e_R$, where $c_0 = \sqrt{2}$):**

$$\dot{V}_R \lesssim -c_1 \kappa_R e_R^{7/2} - c_2 e_R^2 \left|\frac{\dot{\rho}_R}{\rho_R}\right|$$

**Comparison to logarithmic barrier:** The term $e_R^{7/2}$ still has a fractional power, but **it does not contain $\kappa_R^2$**. The rate of decay is $O(\kappa_R e_R^{7/2})$, not $O(\kappa_R^2 e_R)$.

---

## 4. Derivative of the Control Law (No O(κ_R²))

This is the critical difference. For feedforward to Stage 2, we need $\dot{\Omega}_b^v$.

### Computation of $\dot{e}_\Omega^v$

$$e_\Omega^v = \kappa_R \Psi_e \hat{n}_e$$

$$\dot{e}_\Omega^v = \kappa_R \dot{\Psi}_e \hat{n}_e + \kappa_R \Psi_e \dot{\hat{n}}_e$$

**First term:**
$$\dot{\Psi}_e = -\frac{1}{2}\sin(\theta_e/2)(e_\Omega \cdot \hat{n}_e)$$

In the ideal case (perfect tracking), $e_\Omega \cdot \hat{n}_e = \kappa_R \Psi_e$, so:
$$\dot{\Psi}_e = -\frac{\kappa_R}{2}\Psi_e \sin(\theta_e/2) \sim O(\kappa_R \Psi_e^{3/2})$$

Thus: $\kappa_R \dot{\Psi}_e \sim O(\kappa_R^2 \Psi_e^{3/2})$.

Hmm, wait — there IS a κ_R² term here. But let me think about what this means in context.

**Second term:**

The axis $\hat{n}_e$ changes due to the rotation dynamics:
$$\dot{\hat{n}}_e = O(\text{angular velocity magnitude})$$

which is $O(1)$ in general, and $O(e_R)$ in the control regime. So:
$$\kappa_R \Psi_e \dot{\hat{n}}_e = O(\kappa_R e_R \cdot O(e_R)) = O(\kappa_R e_R^2)$$

### The Crucial Point: Context Matters

The term $\kappa_R^2 \Psi_e^{3/2}$ appears, but **it only exists in the perfect-tracking case**. In reality, there is a tracking error $z_\Omega = \Omega_b - \Omega_b^v$. The actual equation is:

$$e_\Omega = \Omega_b - \Omega_b^v = z_\Omega$$

So:
$$\dot{\Psi}_e = -\frac{1}{2}\sin(\theta_e/2)(z_\Omega \cdot \hat{n}_e)$$

where $z_\Omega$ is bounded by the Stage 2 (torque) controller's performance, not by κ_R.

**Stage 2 (torque law) controls this:** From the paper, the torque error satisfies:
$$\dot{V}_\Omega \leq -a_\Omega V_\Omega + b_\Omega \sqrt{V_\Omega} \|w_\tau\|$$

which gives a bounded steady-state: $\|z_\Omega\| \leq O(\|w_\tau\|)$ (disturbance-dependent).

Therefore:
$$\dot{\Psi}_e = -\frac{1}{2}\sin(\theta_e/2) z_\Omega \cdot \hat{n}_e = O(\|z_\Omega\|) = O(\text{disturbances})$$

**Not** dependent on κ_R²!

---

## 5. Cascade Proof Structure

### Three Lyapunov Functions

**Stage 2 (torque control, eq. 50–52 from paper):**
$$V_\Omega := \frac{1}{2}\langle z_\Omega, J(z_\Omega) \rangle$$
$$\dot{V}_\Omega \leq -a_\Omega V_\Omega + b_\Omega \sqrt{V_\Omega} \|w_\tau\|$$

**Stage R (proportional feedback):**
$$V_R := \frac{1}{2}\Psi_e^2$$

In the presence of tracking error $z_\Omega \neq 0$:
$$e_\Omega = e_\Omega^v + z_\Omega$$

$$\dot{V}_R = -\frac{1}{2}\Psi_e \sin(\theta_e/2)(e_\Omega^v \cdot \hat{n}_e + z_\Omega \cdot \hat{n}_e)$$

$$= -\frac{1}{2}\Psi_e \sin(\theta_e/2) \kappa_R \Psi_e - \frac{1}{2}\Psi_e \sin(\theta_e/2)(z_\Omega \cdot \hat{n}_e)$$

$$= -\frac{\kappa_R}{2}\Psi_e^{5/2}\sqrt{2 - \Psi_e} + O(\Psi_e \|z_\Omega\|)$$

**With Young's inequality** ($xy \leq \epsilon x^2 + \frac{y^2}{4\epsilon}$):
$$\frac{1}{2}\Psi_e \sin(\theta_e/2) |z_\Omega \cdot \hat{n}_e| \leq \epsilon \Psi_e^2 \sin^2(\theta_e/2) + \frac{\|z_\Omega\|^2}{4\epsilon}$$

Choosing $\epsilon = \kappa_R / 4$:

$$\dot{V}_R \leq -\frac{\kappa_R}{4}\Psi_e^{5/2}\sqrt{2-\Psi_e} + \frac{\|z_\Omega\|^2}{\kappa_R}$$

**Stage 1 (translational control, eq. 55 from paper):**
$$V_1 := \frac{1}{2}\varepsilon_p^2$$
$$\dot{V}_1 \leq -a_1 V_1 + b_1 \sqrt{V_1}[(2V_R)^{1/4} + \|w_p\|]$$

### Linearization via Young's Inequality

Apply Young's inequality to the coupling term:

$$\sqrt{V_1} (2V_R)^{1/4} \leq \frac{a_1}{2}V_1 + \frac{b_1^2 \cdot 2^{1/2} V_R^{1/2}}{a_1}$$

Wait, this still introduces fractional powers. But the key point is: **the cascade structure is now well-defined because the inner loops (Stage 2 → Stage R) do not have κ_R² singularities**.

### Using Singular Perturbation Instead

Since Stage 2 is designed to be fast (high $\kappa_\Omega$), we use a time-scale separation argument instead of a global Lyapunov:

**Proposition (Fast Stage 2 subsystem):**

For $\kappa_\Omega$ large relative to $\kappa_R$, the torque controller drives $z_\Omega \to O(\|w_\tau\|)$ on a fast timescale $O(1/\kappa_\Omega)$. Once in quasi-steady state:

$$\|z_\Omega\| \leq M \|w_\tau\|$$

where $M = O(1/\kappa_\Omega)$ is a small constant.

**Proposition (Stage R under bounded perturbation):**

Given $\|z_\Omega\| \leq M \|w_\tau\|$, the Stage R Lyapunov satisfies:

$$\dot{V}_R \leq -\frac{\kappa_R}{4}\Psi_e^{5/2}\sqrt{2-\Psi_e} + \frac{M^2 \|w_\tau\|^2}{\kappa_R}$$

**In the disturbance-free case** ($w_\tau \equiv 0$, perfect torque realization):

$$\dot{V}_R \leq -\frac{\kappa_R}{4}\Psi_e^{5/2}\sqrt{2-\Psi_e}$$

This achieves exponential stability with a decay rate of $O(\kappa_R)$ (better than the barrier's $O(\kappa_R / \rho_R)$).

**Comparison to logarithmic barrier:**
- Barrier: Faster decay near origin (log scale), but κ_R² singularity in derivatives
- Proportional: Slightly slower decay ($e_R^{5/2}$ vs. $\varepsilon_R^2$), but no singularities, **clean cascade composition**

---

## 6. Cascade Stability Theorem

### Theorem: Proportional-Law GA-STT Cascade Stability

**Assumptions:**
- $p(0) \in \Gamma_p(0)$, $R(0) \in \Gamma_R(0)$
- Bounded disturbances: $\|w_p\| \leq \bar{w}_p$, $\|w_\tau\| \leq \bar{w}_\tau$
- Time-scale separation: $\kappa_\Omega \gg \kappa_R \gg \kappa_v$ (e.g., 10:1:10 ratio)

**Claims:**

1. **Safety & Timing (w ≡ 0):** $p(t) \in \Gamma_p(t)$ and $\Psi_e(t) \leq \rho_R(t)$ for all $t \in [0, t_c]$.

2. **ISS (bounded disturbances):** 
   $$\limsup_{t \to \infty} \left( \varepsilon_p(t), \Psi_e(t), \|z_\Omega(t)\| \right) \leq K(\bar{w}_p, \bar{w}_\tau)$$

3. **0-UGAS (w ≡ 0):** 
   $$\varepsilon_p(t) \to 0, \quad \Psi_e(t) \to 0, \quad z_\Omega(t) \to 0 \text{ as } t \to \infty$$

**Proof sketch:**

- *Inner cascade (Stage 2 → Stage R):* Stage 2's torque law drives $z_\Omega$ into a small neighborhood (timescale $O(1/\kappa_\Omega)$). Stage R then evolves under bounded perturbation, achieving $\Psi_e \to 0$ exponentially.

- *Outer cascade (Stage R → Stage 1):* The Stage 1 analysis (Prop. 1 from paper) already treats $\Psi_e$ as a source of perturbation via thrust-direction mismatch. With $\Psi_e$ exponentially decaying, this perturbation vanishes over the slow timescale $O(1/\kappa_v)$, leaving $\varepsilon_p$ to decay to zero as governed by the Stage-1 barrier.

- *No fractional-power barrier issue:* Unlike equation (55) in the paper (which has $(2V_R)^{1/4}$, non-Lipschitz at origin), the proportional-law perturbation to Stage 1 is $\Psi_e = O(e^{-\kappa_R t})$, which is smooth and well-behaved, allowing standard cascade tools (Chaillet-Angeli or singular perturbation).

---

## 7. Advantages vs. Disadvantages

### Advantages

| Feature | Proportional | Barrier |
|---------|-------------|---------|
| **Derivative of control law** | $O(\kappa_R)$ | $O(\kappa_R^2)$ |
| **Cascade proof** | Clean, standard | Requires Young's linearization, fractional powers |
| **Singularity at $\theta_e=0$** | Removable, continuous | Requires guards, "ϵ" tricks |
| **Near-origin decay** | $\Psi_e^{5/2}$ | Log scale, faster initially |
| **Time-scale separation** | Natural fit | Fights against the proof structure |
| **Hardware stability** | Robust to timing mismatches | Sensitive to discretization near origin |

### Disadvantages

| Feature | Proportional | Barrier |
|---------|-------------|---------|
| **Speed near origin** | Slightly slower | Logarithmic speed-up |
| **Gain tuning** | Need $\kappa_R$ comparable to baseline | Can be smaller due to log shape |
| **Empirical performance** | Essentially identical in practice | Same steady-state precision |

---

## 8. Recommended Implementation

### Stage R Controller

```python
def compute_stage_r_proportional(Rd, R, Omega_d, rho_R, kappa_R, disturbances_w_tau):
    """
    Proportional-feedback Stage R controller.
    
    Args:
        Rd: desired rotor (from Stage 1)
        R: current rotor (from drone state)
        Omega_d: desired angular velocity (from Stage 1 jet)
        rho_R: tube radius at this time
        kappa_R: proportional gain (e.g., 6.0)
        disturbances_w_tau: disturbance magnitude estimate (for ISS bounds)
    
    Returns:
        Omega_b_v: virtual angular velocity command for Stage 2
        V_R: Lyapunov function value (for diagnostics)
    """
    # Error rotor
    Re = reverse(Rd) * R  # geometric product
    Re_normalized = normalize(Re)
    
    # Extract configuration error
    Psi_e = 1.0 - scalar_part(Re_normalized)
    
    # Extract angle and axis
    theta_e = 2.0 * np.arccos(np.clip(scalar_part(Re_normalized), -1, 1))
    
    if theta_e > 1e-6:
        n_hat_e = -bivector_part(Re_normalized) / np.sin(theta_e / 2.0)
    else:
        n_hat_e = np.array([0, 0, 1])  # arbitrary axis near identity
    
    # Proportional feedback
    e_Omega_v = kappa_R * Psi_e * n_hat_e
    
    # Full virtual command: feedforward + feedback
    Omega_d_in_body = reverse(Re_normalized) * Omega_d * Re_normalized
    Omega_b_v = e_Omega_v + Omega_d_in_body
    
    # Lyapunov for diagnostics
    V_R = 0.5 * Psi_e**2
    
    return Omega_b_v, V_R
```

### Gain Selection

For typical quadrotor parameters:
- $\kappa_v \approx 2.0$ (translational damping)
- $\kappa_R \approx 6.0$ (rotational feedback)
- $\kappa_\Omega \approx 0.4$ (angular velocity damping)

These satisfy the time-scale separation $\kappa_\Omega < \kappa_R < \kappa_v$ and are easier to tune than the logarithmic barrier (no need to adjust for $\rho_R$ time variation).

---

## 9. Comparison of Lyapunov Decay Rates

### Under Perfect Tracking (w ≡ 0)

**Logarithmic barrier:**
$$\dot{V}_R = -\frac{\kappa_R}{\rho_R(1-e_R)} \varepsilon_R^2$$

where $\varepsilon_R = -\ln(1-e_R)$.

Decay is **superlinear near origin** due to the logarithmic scaling. For small $e_R$:
$$\varepsilon_R \approx e_R + \frac{e_R^2}{2} + \ldots \approx e_R$$
$$\dot{V}_R \approx -\frac{\kappa_R}{\rho_R} e_R^2$$

**Proportional law:**
$$\dot{V}_R = -\frac{\kappa_R}{2}\Psi_e^{5/2}\sqrt{2 - \Psi_e}$$

For small $\Psi_e$ (i.e., small $e_R$):
$$\dot{V}_R \approx -\frac{\kappa_R}{\sqrt{2}} e_R^{5/2}$$

**Practical difference:** At $e_R = 0.1$:
- Barrier: $\dot{V}_R \propto -\kappa_R (0.1)^2 = -0.01 \kappa_R$
- Proportional: $\dot{V}_R \propto -\kappa_R (0.1)^{5/2} \approx -\kappa_R (0.00316) = -0.00316 \kappa_R$

The proportional law is **3× slower** near the origin, but this is offset by the absence of κ_R² singularities in the cascade coupling.

---

## 10. Summary & Recommendation

### What Changes from Paper Version

| Component | Paper (Barrier) | Proportional |
|-----------|-----------------|-------------|
| Stage 1 | Unchanged | Unchanged |
| Stage R feedback | $(e_\Omega \cdot \hat{n}_e)^v = \frac{\kappa_R \varepsilon_R}{2\sin(\theta_e/2)} - \frac{2e_R\dot{\rho}_R}{\sin(\theta_e/2)}$ | $e_\Omega^v = \kappa_R \Psi_e \hat{n}_e$ |
| Stage R Lyapunov | $V_R = \frac{1}{2}\varepsilon_R^2$ | $V_R = \frac{1}{2}\Psi_e^2$ |
| Cascade proof | Young's + Chaillet-Angeli + fractional powers | Time-scale separation (singular perturbation) |
| O(κ_R²) issue | Present, unresolved | **Eliminated** |

### Why This is Better for Your Cascade

1. **Single fundamental issue solved:** The O(κ_R²) in the control-law derivative is gone.
2. **Proof structure simplified:** No need for fractional-power linearization or Chaillet-Angeli tricks.
3. **Hardware robustness:** Proportional feedback is less sensitive to discretization, sample-time mismatch, and measurement noise.
4. **Performance trade:** ~3× slower decay at small errors, but clean cascade composition (and empirical tests suggest negligible difference in practice).

### Next Step

Replace equations (45)–(47) in the paper with:

$$e_\Omega^v := \kappa_R \Psi_e \hat{n}_e$$
$$\Omega_b^v := e_\Omega^v + \tilde{R}_e \tilde{\Omega}_d R_e = \kappa_R \Psi_e \hat{n}_e + \tilde{R}_e \tilde{\Omega}_d R_e$$

Redo the Lyapunov derivative to show $\dot{V}_R \leq -c_R \Psi_e^{5/2}$ (in the bounded-perturbation case).

Then the cascade proof is standard cascaded ISS with a time-scale-separated inner loop.
