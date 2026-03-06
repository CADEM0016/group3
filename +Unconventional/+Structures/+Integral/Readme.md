**🏗 +Structures Module**



**Overview**



The +Structures package contains the physics-based wing structural sizing model integrated into the aircraft MDO framework.



This module replaces empirical wing weight estimation with a load-driven structural analysis approach consistent with CS-25 manoeuvre requirements.



The structural model is fully coupled with:



* Aircraft geometry
* Mission fuel weight
* MTOM iteration loop
* Empty weight calculation



The wing structural mass is computed dynamically and fed back into the main sizing loop.



**📦 Module Architecture**



+Structures/

│

├── WingLoads.m

├── WingSizing.m

└── WingWeightPhysics.m



**🔹 1️⃣ WingLoads.m**



**Purpose**

Computes spanwise aerodynamic loads and internal forces.



**Inputs**

* ac.MTOM
* Wing geometry (ac.geom.b, ac.geom.S, etc.)



**Outputs**



* Lift distribution
* Shear force distribution
* Bending moment distribution
* Root bending moment (M\_root)
* Root shear (V\_root)



**Assumptions**



* Elliptical lift distribution
* 2.5g manoeuvre load case
* Symmetric loading
* Half-wing integration



This function provides the structural load state used for sizing.



**🔹 2️⃣ WingSizing.m**



**Purpose**



Sizes primary load-carrying wing box components.



**Inputs**



* Root bending moment
* Wing geometry
* Material properties
* Structural Model
* Two-spar wing box
* Skin-dominated bending response
* Closed-section approximation
* Checks Performed
* Bending stress constraint
* Compression skin buckling check



**Outputs**



* Required skin thickness
* Estimated structural mass
* Intermediate sizing parameters



This function converts loads into structural dimensions.



**🔹 3️⃣ WingWeightPhysics.m**



**Purpose**



High-level wrapper function.



**Workflow**



* Calls WingLoads
* Calls WingSizing
* Integrates spanwise structural mass
* Returns total wing structural weight



**Output**

W\_wing



This value is passed to:

+cast/MassObj.m

and included in the empty weight computation.



**🔁 Coupling Within MDO Framework**



The structural module participates in the MTOM iteration loop:



* MTOM estimate
* Wing loads computed
* Structure sized
* Wing mass updated
* Empty weight updated
* MTOM recalculated
* Loop until convergence



This creates nonlinear feedback between structure and aircraft mass.



**⚙️ Design Assumptions**



* Aluminium baseline material
* Elastic structural behaviour
* Conceptual-level fidelity
* No detailed rib modelling
* No aeroelastic coupling
* No fatigue or damage tolerance modelling



The model is suitable for conceptual trade studies and optimisation.



**🧠 Engineering Scope**



Included:



* Load-based structural sizing
* Buckling stability check
* Structural mass integration
* Full MDO coupling



Not Included:



* Detailed finite element modelling
* Torsional stiffness modelling
* Gust load modelling (future extension)
* Composite laminate modelling (future extension)



🚀 Future Development



Planned improvements:



* Gust load case integration
* Fuel weight distribution modelling
* Spanwise variable thickness
* Composite material option
* Aeroelastic deflection constraint
* Structural optimisation variables



**👤 Discipline Ownership**



This module is owned by the Wing Structures discipline.



Any modifications affecting:



* Wing mass
* Load cases
* Structural constraints
* Material modelling



must be coordinated with the systems lead to preserve MTOM convergence stability.

