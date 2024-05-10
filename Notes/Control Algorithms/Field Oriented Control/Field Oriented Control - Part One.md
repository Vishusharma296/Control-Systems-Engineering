## Field Oriented Control (FOC): 

### Summary

**Field Oriented Control (FOC)** is a ***Vector Control*** technique used for the precise control of electric motors, particularly AC induction motors and Permanent Magnet Synchronous Motors (PMSMs), to achieve high-performance torque and speed regulation. It is a sophisticated method that optimizes motor performance through advanced mathematical transformations and closed-loop control strategies. Other vector control techniques commonly used in VFD are Direct Torque Control (**DTC**) and the Model Predictive Control (**MPC**).

FOC can be broadly classified into two categories: 
1. Direct FOC with feedback vector control
2. Indirect FOC with feedforward vector control (Sensorless Control)
### Principle of Operation

1. **Transformation to a Rotating Reference Frame:** FOC involves transforming the three-phase AC motor currents and voltages from the stationary phase frame (a-b-c) to a rotating reference frame (d-q) aligned with the rotor flux vector:
   - **Clarke Transformation:** Converts the three-phase currents ($i_a$ , $i_b$, $i_c$) into a two-component vector ($i_\alpha$ , $i_\beta$) in a stationary reference frame.
   - **Park Transformation:** Rotates the resulting vector to a reference frame aligned with the rotor flux (d-axis along rotor flux, q-axis along torque).

3. **Decoupled Control:**
   After transformation, the motor control becomes decoupled into two independent control loops:
   - **Flux Control (q-axis):** Regulates the magnetizing flux by controlling the q-axis current **($i_q$)** perpendicular to rotor flux.
   - **Torque Control (d-axis):** Regulates the torque by controlling the d-axis current **($i_d$)** aligned with rotor flux.

3. **Current Control:**
   - **PI Controllers:** Use Proportional-Integral (PI) controllers to regulate $i_d$ and $i_q$ based on their reference values.
   - **Inverse Park and Clarke Transformations:** Convert control signals $v_d$ , $v_q$ back to the stationary reference frame using inverse transformations to generate motor voltage commands.

### Advantages of FOC

- **Improved efficiency and accuracy:** Enables precise control of torque and flux, enhancing motor efficiency.
- **Reduced Torque Ripple:** Minimizes torque fluctuations, leading to smoother motor operation.
- **Wide Speed Range:** Allows stable operation across varying speeds, including low-speed operation.
### Applications

FOC is extensively used in:
- Power efficient drive systems
- Applications requiring high-performance motor control
- Electric vehicles (EVs)
- Industrial drives
- Robotics
### Direct Quadrature (d-q) Coordinate Transformation

- The d-q transformation is a mathematical technique used to convert three-phase AC quantities into two-phase DC-like quantities, simplifying the analysis and control of electrical machines. 

- The governing equations for the d-q coordinate transformation can be expressed using the Clarke and Park transformations. The Clarke transformation converts the three-phase currents $i_a$, $i_b$ , and $i_c$ into a two-component vector ($i_\alpha$ , $i_\beta$) in a stationary reference frame. The Park transformation then rotates this vector to a reference frame aligned with the rotor flux.

- In d-q coordinates, the d-axis is aligned with the rotor flux (flux-linkage component of the current), and the q-axis is (perpendicular to the d-axis) aligned with the torque component of the current. The transformation allows for the decoupling of the machine's torque and flux control, enabling independent and precise control of these two parameters. 

- For a balanced three phase system: $i_a +i_b +i_c = 0$. So one of the three current components is redundant. 

#### Direct Quadrature Zero Transformation (DQZ)

DQZ transformation is the combination of Clark and Park transformation.

Clark Matrix is given by:

$$K_{C} = \sqrt{\frac{2}{3}}\cdot\begin{bmatrix}
   1 & -\frac{1}{2} & -\frac{1}{2} \\
   0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} \\
   \frac{1}{\sqrt{2}} & \frac{1}{\sqrt{2}} & \frac{1}{\sqrt{2}}
\end{bmatrix}$$
Park Matrix is given by:
$$K_{P} = \begin{bmatrix}
   \cos{\left(\theta\right)} & \sin{\left(\theta\right)} & 0 \\
   -\sin{\left(\theta\right)} & \cos{\left(\theta\right)} & 0 \\
   0 & 0 & 1
\end{bmatrix}$$
Combined Clark and Park Matrix can be written as:
$$K_{CP} = K_{P}\cdot K_{C}$$

$$ K_{CP} = \sqrt{\frac{2}{3}}\begin{bmatrix}
   \cos{\left(\theta\right)}
      & \cos{\left(\theta - \frac{2\pi}{3}\right)}
      & \cos{\left(\theta + \frac{2\pi}{3}\right)} \\
   -\sin{\left(\theta\right)}
      & -\sin{\left(\theta - \frac{2\pi}{3}\right)}
      & -\sin{\left(\theta + \frac{2\pi}{3}\right)} \\
   \frac{\sqrt{2}}{2}
      & \frac{\sqrt{2}}{2}
      & \frac{\sqrt{2}}{2}
\end{bmatrix}$$
The inverse Clark Park transform is
$$K_{CP}^{-1} = \sqrt{\frac{2}{3}}\begin{bmatrix}
   \cos{\left(\theta\right)}
      & -\sin{\left(\theta\right)}
      & \frac{\sqrt{2}}{2} \\
   \cos{\left(\theta - \frac{2\pi}{3}\right)}
      & -\sin{\left(\theta - \frac{2\pi}{3}\right)}
      & \frac{\sqrt{2}}{2} \\
   \cos{\left(\theta + \frac{2\pi}{3}\right)}
      & -\sin{\left(\theta + \frac{2\pi}{3}\right)}
      & \frac{\sqrt{2}}{2}
\end{bmatrix}$$

#### Clarke Transformation (a-b-c to α-β Frame):

$$
\begin{bmatrix} 
i_\alpha \\ 
i_\beta 
\end{bmatrix} 
= 
K_c
\begin{bmatrix} 
i_a \\ 
i_b \\ 
i_c 
\end{bmatrix} 
$$
$$
K_c = \begin{bmatrix} 
1 & -\frac{1}{2} & -\frac{1}{2} \\ 
0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} 
\end{bmatrix} 
$$
$$ 
\begin{bmatrix} i_\alpha \\ i_\beta \end{bmatrix} = \begin{bmatrix} 1 & -\frac{1}{2} & -\frac{1}{2} \\ 0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} \end{bmatrix} \begin{bmatrix} i_a \\ i_b \\ i_c \end{bmatrix}
$$

#### Park Transformation (α-β to d-q Frame):
$$
\begin{bmatrix} 
i_d \\ 
i_q 
\end{bmatrix} 
= 
K_p
\begin{bmatrix} 
i_\alpha \\ 
i_\beta 
\end{bmatrix} $$

$$
K_p = \begin{bmatrix} 
\cos(\theta) & -\sin(\theta) \\ 
\sin(\theta) & \cos(\theta) 
\end{bmatrix}$$
$$
\begin{bmatrix} i_d \\ i_q \end{bmatrix} = \begin{bmatrix} \cos(\theta) & -\sin(\theta) \\ \sin(\theta) & \cos(\theta) \end{bmatrix} \begin{bmatrix} i_\alpha \\ i_\beta \end{bmatrix}
$$

### Combined Transformation (a-b-c to d-q Frame):

$$ 
\begin{bmatrix} 
i_d \\ 
i_q 
\end{bmatrix} 
= 
K_p K_c
\begin{bmatrix} 
i_a \\ 
i_b \\ 
i_c 
\end{bmatrix} 
$$
$$
\begin{bmatrix} 
i_d \\ 
i_q 
\end{bmatrix} 
= 
\begin{bmatrix} 
\cos(\theta) & -\sin(\theta) \\ 
\sin(\theta) & \cos(\theta) 
\end{bmatrix} 
\begin{bmatrix} 
1 & -\frac{1}{2} & -\frac{1}{2} \\ 
0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} 
\end{bmatrix} 
\begin{bmatrix} 
i_a \\ 
i_b \\ 
i_c 
\end{bmatrix} 
$$
These equations illustrate the process of transforming currents between the stationary a-b-c frame and the rotating d-q frame, facilitating independent control of torque and flux in field-oriented control of AC motors. This transformation is essential for implementing Field Oriented Control (FOC) algorithms in AC motor drives to achieve precise control of torque and flux. The inverse transformations can also be defined to convert the d-q currents $i_d$ and $i_q$ back to the three-phase stationary reference frame $i_a$, $i_b$ , $i_c$. Here $\theta$ is the electrical angle of the rotor.

### Kron Equations

Kron equations are a set of equations used to model the dynamic behavior of electrical machines in the d-q reference frame for Field Oriented Control (FOC). The Kron equations, along with appropriate mechanical and control equations, form the basis for designing FOC algorithms and controllers. These equations are essential for simulating and analyzing the performance of motor control systems.
#### Basic Motor Equations For Permanent Magnet Synchronous Motor  (PMSM)

**Voltage Equations (d-q Frame, synchronous rotating frame):**
   $$v_d = R_s i_d + L_d \frac{di_d}{dt} - \omega_e L_q i_q$$
   $$v_q = R_s i_q + L_q \frac{di_q}{dt} + \omega_e L_d i_d   $$
   
   - $v_d$ and $v_q$ are the dq-axis stator voltages,
   - $i_d$ and $i_q$ are the dq-axis stator currents,
   - $R_s$ is the stator resistance,
   - $L_d$ and $L_q$ are the d-axis and q-axis stator inductances respectively,
   - $\omega_e$ is the electrical angular velocity (rad/s).
   - $\omega_r$ is the angular velocity of the rotor (rad/s).

**Flux Linkage Equations**

These equations define the d-q axis flux linkages ($\Psi_d$, $\Psi_q$) in terms of the d-q axis currents ($i_d$, $i_q$), d-q axis inductances ($L_d$, $L_q$), and the permanent magnet flux linkage ($\Psi_f$)

$$ \Psi_d = L_d i_d + \Psi_f$$
$$ \Psi_q = L_q i_q$$

**Torque Equation:**

This equation calculates the electromagnetic torque ($T_e$) produced by the machine, based on the d-q axis flux linkages, currents, and the number of pole pairs ($P$)
   $$T_e = \frac{3}{2} \frac{P}{2} (\Psi_f i_q)$$
      $$T_e = \frac{3}{2} \frac{P}{2} (\Psi_d i_q - \Psi_q i_d)$$
   
   - $T_e$ is the electromagnetic torque,
   - $P$ is the number of pole pairs,
   - $\Psi_f$ is the permanent magnet flux linkage.

**Electrical Angular Velocity (Rotor Speed) Equation:**
$$\omega_e = \frac{P}{2} \omega_m$$
   
   - $\omega_m$ is the mechanical angular velocity (motor speed).
   - $\omega_r$ is the electrical rotor speed (state)

**Mechanical Equations:**

$$ T_e = T_L + J \frac{d\omega_m}{dt}$$

$$ T_e = T_L + \frac{2}{P}J \frac{d\omega_m}{dt}$$
- $T_L$ is the load Torque
- $J$ is the rotor inertia

### State Space Equations

The dynamic model equations for Field Oriented Control (FOC) in state-space form are:


$$\frac{di_d}{dt} = \frac{1}{L_d}\left(v_d - R_si_d + \omega_r L_q i_q\right)$$
$$\frac{di_q}{dt} = \frac{1}{L_q}\left(v_q - R_si_q - \omega_r\left(L_di_d + \psi_f\right)\right)$$
$$\frac{d\omega_r}{dt} = \frac{3P}{2J}\left(\frac{\psi_f}{L_d}i_q + \left(1-\frac{L_q}{L_d}\right)i_di_q\right) - \frac{B}{J}\omega_r - \frac{T_L}{J}$$


These state-space equations describe the dynamic behavior of the electrical machine in the d-q reference frame, relating the currents, voltages, flux linkages, torque, and rotor speed. They form the basis for implementing FOC algorithms and control strategies.
### Control of Machine

- **Rotor Position and Speed Estimation:** In practice, sensorless control methods often use observers or estimators to determine the rotor position and speed (needed for the transformation between a-b-c and d-q frames) based on the d-q currents and voltage.

- **LTI system:** Clark and Park transformation (d-q coordinate transformation) converts the three phase currents and voltages into a two coordinate Linear Time Invariant (**LTI**) system. This transformation enables the implementation of PI controllers for speed and torque control.

- **Controllers**: Proportional-Integral (PI) or State-Space controllers are used to regulate the d-q currents $i_d$ and $i_q$ based on desired torque and speed references, enabling precise control of PMSMs in various applications such as electric vehicles, industrial drives, and the robotics.

- **Sampling Frequency:** FOC is usually implemented with a sampling frequency of 20 kHz for the current control loop and 1 kHz for the speed control loop.


### Resources for Further Learning

1. [NXP Community Model based Design and FOC Theory](https://community.nxp.com/t5/Model-Based-Design-Toolbox-MBDT/Module-2-PMSM-and-FOC-Theory/m-p/761926)
2. [Imperix FOC for PMSM](https://imperix.com/doc/implementation/field-oriented-control-of-pmsm)
3. [Dave Wilson, Texas Instruments FOC of PMSM](https://www.youtube.com/watch?v=cdiZUszYLiA)
4. [Great Scott-Trinamic, FOC for BLDC Motor](https://www.youtube.com/watch?v=Nhy6g9wGHow)
5. [Implementation of field oriented control for permanent magnet synchronous motor](https://ieeexplore.ieee.org/document/7045278)



