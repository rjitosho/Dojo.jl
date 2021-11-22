<!-- These are atlas torso and limb models, for full Atlas robot model, please see
     xacro files containing fully assembed robots in atlas_description/robots/ -->
<robot name="drc_skeleton">
    <link name="l_clav">
        <inertial>
            <mass value="2.369" />
            <origin xyz="0.014 0.058 0.029" rpy="1.04719755 0 0" />
            <inertia ixx="0.004" ixy="0.001" ixz="0" iyy="0.006" iyz="0" izz="0.007" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_clav.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_clav.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_farm">
        <inertial>
            <mass value="0.981" />
            <origin xyz="0 0.041 0" rpy="0 -0 0" />
            <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.003" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_farm.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_farm.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_foot">
        <inertial>
            <mass value="1.634" />
            <origin xyz="0.027 0 -0.067" rpy="0 -0 0" />
            <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.007" iyz="0" izz="0.008" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_foot.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_foot.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_hand">
        <inertial>
            <mass value="2.263" />
            <origin xyz="0 0.093 0" rpy="0 -0 0" />
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.01" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_hand.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_hand.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_larm">
        <inertial>
            <mass value="2.148" />
            <origin xyz="-0.003 0.099 -0.014" rpy="0 -0 0" />
            <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.006" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_larm.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_larm.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_lglut">
        <inertial>
            <mass value="0.69" />
            <origin xyz="0.0133341 0.0170484 -0.0312052" rpy="0 -0 0" />
            <inertia ixx="0.000691326" ixy="-2.24344e-05" ixz="2.50508e-06" iyy="0.00126856" iyz="0.000137862" izz="0.00106487" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_lglut.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_lglut.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_lleg">
        <inertial>
            <mass value="4.367" />
            <origin xyz="0.001 0 -0.187" rpy="0 -0 0" />
            <inertia ixx="0.077" ixy="0" ixz="-0.003" iyy="0.076" iyz="0" izz="0.01" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_lleg.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_lleg.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_scap">
        <inertial>
            <mass value="2.707" />
            <origin xyz="-0.002 0.108 0" rpy="0 -0 0" />
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.013" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_scap.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_scap.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_talus">
        <inertial>
            <mass value="0.1" />
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <inertia ixx="1.01674e-05" ixy="0" ixz="0" iyy="8.42775e-06" iyz="0" izz="1.30101e-05" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_talus.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_talus.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_uarm">
        <inertial>
            <mass value="1.881" />
            <origin xyz="0.007 0.114 0.008" rpy="0 -0 0" />
            <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.003" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_uarm.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_uarm.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_uglut">
        <inertial>
            <mass value="0.5166" />
            <origin xyz="0.00529262 -0.00344732 0.00313046" rpy="0 -0 0" />
            <inertia ixx="0.00074276" ixy="-3.79607e-08" ixz="-2.79549e-05" iyy="0.000688179" iyz="-3.2735e-08" izz="0.00041242" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_uglut.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_uglut.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="l_uleg">
        <inertial>
            <mass value="7.34" />
            <origin xyz="0 0 -0.21" rpy="0 -0 0" />
            <inertia ixx="0.09" ixy="0" ixz="0" iyy="0.09" iyz="0" izz="0.02" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_uleg.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/l_uleg.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="ltorso">
        <inertial>
            <mass value="1.92" />
            <origin xyz="-0.0112984 -3.15366e-06 0.0746835" rpy="0 -0 0" />
            <inertia ixx="0.0039092" ixy="-5.04491e-08" ixz="-0.000342157" iyy="0.00341694" iyz="4.87119e-07" izz="0.00174492" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/ltorso.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/ltorso.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="mtorso">
        <inertial>
            <mass value="0.55" />
            <origin xyz="-0.00816266 -0.0131245 0.0305974" rpy="0 -0 0" />
            <inertia ixx="0.000454181" ixy="-6.10764e-05" ixz="3.94009e-05" iyy="0.000483282" iyz="5.27463e-05" izz="0.000444215" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/mtorso.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/mtorso.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="pelvis">
        <inertial>
            <mass value="14.2529" />
            <origin xyz="0.0111 0 0.0271" rpy="0 -0 0" />
            <inertia ixx="0.1244" ixy="0.0008" ixz="-0.0007" iyy="0.0958" iyz="-0.0005" izz="0.1167" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/pelvis.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/pelvis.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_clav">
        <inertial>
            <mass value="2.369" />
            <origin xyz="0.014 -0.058 0.029" rpy="-1.04719755 0 0" />
            <inertia ixx="0.004" ixy="-0.001" ixz="0" iyy="0.006" iyz="0" izz="0.007" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_clav.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_clav.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_farm">
        <inertial>
            <mass value="0.981" />
            <origin xyz="0 -0.041 0" rpy="0 -0 0" />
            <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.003" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_farm.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_farm.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_foot">
        <inertial>
            <mass value="1.634" />
            <origin xyz="0.027 0 -0.067" rpy="0 -0 0" />
            <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.007" iyz="0" izz="0.008" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_foot.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_foot.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_hand">
        <inertial>
            <mass value="2.263" />
            <origin xyz="0 -0.093 0" rpy="0 -0 0" />
            <inertia ixx="0.01" ixy="0" ixz="-0" iyy="0.002" iyz="0" izz="0.01" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_hand.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_hand.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_larm">
        <inertial>
            <mass value="2.148" />
            <origin xyz="-0.003 -0.099 -0.014" rpy="0 -0 0" />
            <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.006" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_larm.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_larm.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_lglut">
        <inertial>
            <mass value="0.69" />
            <origin xyz="0.0133341 -0.0170484 -0.0312052" rpy="0 -0 0" />
            <inertia ixx="0.000691326" ixy="2.24344e-05" ixz="2.50508e-06" iyy="0.00126856" iyz="-0.000137862" izz="0.00106487" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_lglut.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_lglut.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_lleg">
        <inertial>
            <mass value="4.367" />
            <origin xyz="0.001 0 -0.187" rpy="0 -0 0" />
            <inertia ixx="0.077" ixy="-0" ixz="-0.003" iyy="0.076" iyz="-0" izz="0.01" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_lleg.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_lleg.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_scap">
        <inertial>
            <mass value="2.707" />
            <origin xyz="-0.002 -0.108 0" rpy="0 -0 0" />
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.013" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_scap.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_scap.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_talus">
        <inertial>
            <mass value="0.1" />
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <inertia ixx="1.01674e-05" ixy="0" ixz="0" iyy="8.42775e-06" iyz="0" izz="1.30101e-05" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_talus.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_talus.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_uarm">
        <inertial>
            <mass value="1.881" />
            <origin xyz="0.007 -0.114 0.008" rpy="0 -0 0" />
            <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.003" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_uarm.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_uarm.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_uglut">
        <inertial>
            <mass value="0.5166" />
            <origin xyz="0.00529262 0.00344732 0.00313046" rpy="0 -0 0" />
            <inertia ixx="0.00074276" ixy="3.79607e-08" ixz="-2.79549e-05" iyy="0.000688179" iyz="3.2735e-08" izz="0.00041242" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_uglut.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_uglut.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="r_uleg">
        <inertial>
            <mass value="7.34" />
            <origin xyz="0 0 -0.21" rpy="0 -0 0" />
            <inertia ixx="0.09" ixy="0" ixz="0" iyy="0.09" iyz="0" izz="0.02" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_uleg.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/r_uleg.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <link name="utorso">
        <inertial>
            <mass value="18.484" />
            <origin xyz="0.02 -0.001 0.211" rpy="0 -0 0" />
            <!-- original inertia violates triangle rule
            <inertia ixx="0.395" ixy="0" ixz="0.083" iyy="1.089" iyz="-0.003" izz="0.327" />
            -->
            <inertia ixx="0.395" ixy="0" ixz="0.083" iyy="1.089" iyz="-0.003" izz="0.827" />
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/utorso.dae" scale="1 1 1" />
            </geometry>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 -0 0" />
            <geometry>
                <mesh filename="package://atlas_description/meshes/utorso.stl" scale="1 1 1" />
            </geometry>
        </collision>
    </link>
    <joint name="back_lbz" type="revolute">
        <origin xyz="-0.0125 0 0" rpy="0 -0 0" />
        <axis xyz="0 0 1" />
        <parent link="pelvis" />
        <child link="ltorso" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="124.016" velocity="12" lower="-0.610865" upper="0.610865" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.6109" soft_upper_limit="10.6109" />
    </joint>
    <joint name="back_mby" type="revolute">
        <origin xyz="0 0 0.09" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="ltorso" />
        <child link="mtorso" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="206.843" velocity="12" lower="-1.2" upper="1.28" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.2" soft_upper_limit="11.28" />
    </joint>
    <joint name="back_ubx" type="revolute">
        <origin xyz="0 0 0.05" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="mtorso" />
        <child link="utorso" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="94.91" velocity="12" lower="-0.790809" upper="0.790809" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.7908" soft_upper_limit="10.7908" />
    </joint>
    <joint name="l_arm_elx" type="revolute">
        <origin xyz="0 0.121 0.013" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="l_uarm" />
        <child link="l_larm" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="114" velocity="12" lower="0" upper="2.35619" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10" soft_upper_limit="12.3562" />
    </joint>
    <joint name="l_arm_ely" type="revolute">
        <origin xyz="0 0.185 0" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="l_scap" />
        <child link="l_uarm" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="114" velocity="12" lower="0" upper="3.14159" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10" soft_upper_limit="13.1416" />
    </joint>
    <joint name="l_arm_shx" type="revolute">
        <origin xyz="0 0.075 0.036" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="l_clav" />
        <child link="l_scap" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="170" velocity="12" lower="-1.39626" upper="1.74533" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.3963" soft_upper_limit="11.7453" />
    </joint>
    <joint name="l_arm_usy" type="revolute">
        <origin xyz="0.024 0.221 0.289" rpy="0 -0 0" />
        <axis xyz="0 0.5 0.866025" />
        <parent link="utorso" />
        <child link="l_clav" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="212" velocity="12" lower="-1.9635" upper="1.9635" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.9635" soft_upper_limit="11.9635" />
    </joint>
    <joint name="l_arm_mwx" type="revolute">
        <origin xyz="0 0.058 0" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="l_farm" />
        <child link="l_hand" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="60" velocity="12" lower="-0.436" upper="1.571" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.436" soft_upper_limit="11.571" />
    </joint>
    <joint name="l_arm_uwy" type="revolute">
        <origin xyz="0 0.188 -0.013" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="l_larm" />
        <child link="l_farm" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="114" velocity="12" lower="-1.571" upper="1.571" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.571" soft_upper_limit="11.571" />
    </joint>
    <joint name="l_leg_lax" type="revolute">
        <origin xyz="0 0 0" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="l_talus" />
        <child link="l_foot" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="90" velocity="12" lower="-0.436" upper="0.436" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.436" soft_upper_limit="10.436" />
    </joint>
    <joint name="l_leg_lhy" type="revolute">
        <origin xyz="0.05 0 -0.05" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="l_lglut" />
        <child link="l_uleg" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="260" velocity="12" lower="-1.75" upper="0.524" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.75" soft_upper_limit="10.524" />
    </joint>
    <joint name="l_leg_mhx" type="revolute">
        <origin xyz="0 0 0" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="l_uglut" />
        <child link="l_lglut" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="180" velocity="12" lower="-0.47" upper="0.495" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.47" soft_upper_limit="10.495" />
    </joint>
    <joint name="l_leg_uay" type="revolute">
        <origin xyz="0 0 -0.422" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="l_lleg" />
        <child link="l_talus" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="220" velocity="12" lower="-0.698" upper="0.698" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.698" soft_upper_limit="10.698" />
    </joint>
    <joint name="l_leg_uhz" type="revolute">
        <origin xyz="0 0.089 0" rpy="0 -0 0" />
        <axis xyz="0 0 1" />
        <parent link="pelvis" />
        <child link="l_uglut" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="110" velocity="12" lower="-0.32" upper="1.14" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.32" soft_upper_limit="11.14" />
    </joint>
    <joint name="l_leg_kny" type="revolute">
        <origin xyz="-0.05 0 -0.374" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="l_uleg" />
        <child link="l_lleg" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="220" velocity="12" lower="0" upper="2.45" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10" soft_upper_limit="12.45" />
    </joint>
    <joint name="neck_ay" type="revolute">
        <!-- per slides on sensor head kinematics 2012-12-14 -->
        <origin xyz="0.16555 0 0.50823" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="utorso" />
        <child link="head" />
        <dynamics damping="0.1" friction="0" />
        <!-- per slides on sensor head kinematics 2012-12-14 -->
        <limit effort="5" lower="-0.610865238" upper="1.13446401" velocity="12"/>
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.610865238" soft_upper_limit="11.13446401"/>
    </joint>
    <joint name="r_arm_elx" type="revolute">
        <origin xyz="0 -0.121 0.013" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="r_uarm" />
        <child link="r_larm" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="114" velocity="12" lower="-2.35619" upper="0" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-12.3562" soft_upper_limit="10" />
    </joint>
    <joint name="r_arm_ely" type="revolute">
        <origin xyz="0 -0.185 0" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="r_scap" />
        <child link="r_uarm" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="114" velocity="12" lower="0" upper="3.14159" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10" soft_upper_limit="13.1416" />
    </joint>
    <joint name="r_arm_shx" type="revolute">
        <origin xyz="0 -0.075 0.036" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="r_clav" />
        <child link="r_scap" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="170" velocity="12" lower="-1.74533" upper="1.39626" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.7453" soft_upper_limit="11.3963" />
    </joint>
    <joint name="r_arm_usy" type="revolute">
        <origin xyz="0.024 -0.221 0.289" rpy="0 -0 0" />
        <axis xyz="0 0.5 -0.866025" />
        <parent link="utorso" />
        <child link="r_clav" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="212" velocity="12" lower="-1.9635" upper="1.9635" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.9635" soft_upper_limit="11.9635" />
    </joint>
    <joint name="r_arm_mwx" type="revolute">
        <origin xyz="0 -0.058 0" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="r_farm" />
        <child link="r_hand" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="60" velocity="12" lower="-1.571" upper="0.436" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.571" soft_upper_limit="10.436" />
    </joint>
    <joint name="r_arm_uwy" type="revolute">
        <origin xyz="0 -0.188 -0.013" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="r_larm" />
        <child link="r_farm" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="114" velocity="12" lower="-1.571" upper="1.571" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.571" soft_upper_limit="11.571" />
    </joint>
    <joint name="r_leg_lax" type="revolute">
        <origin xyz="0 0 0" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="r_talus" />
        <child link="r_foot" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="90" velocity="12" lower="-0.436" upper="0.436" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.436" soft_upper_limit="10.436" />
    </joint>
    <joint name="r_leg_lhy" type="revolute">
        <origin xyz="0.05 0 -0.05" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="r_lglut" />
        <child link="r_uleg" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="260" velocity="12" lower="-1.745" upper="0.524" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.745" soft_upper_limit="10.524" />
    </joint>
    <joint name="r_leg_mhx" type="revolute">
        <origin xyz="0 0 0" rpy="0 -0 0" />
        <axis xyz="1 0 0" />
        <parent link="r_uglut" />
        <child link="r_lglut" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="180" velocity="12" lower="-0.495" upper="0.47" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.495" soft_upper_limit="10.47" />
    </joint>
    <joint name="r_leg_uay" type="revolute">
        <origin xyz="0 0 -0.422" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="r_lleg" />
        <child link="r_talus" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="220" velocity="12" lower="-0.698" upper="0.698" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10.698" soft_upper_limit="10.698" />
    </joint>
    <joint name="r_leg_uhz" type="revolute">
        <origin xyz="0 -0.089 0" rpy="0 -0 0" />
        <axis xyz="0 0 1" />
        <parent link="pelvis" />
        <child link="r_uglut" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="110" velocity="12" lower="-1.14" upper="0.32" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-11.14" soft_upper_limit="10.32" />
    </joint>
    <joint name="r_leg_kny" type="revolute">
        <origin xyz="-0.05 0 -0.374" rpy="0 -0 0" />
        <axis xyz="0 1 0" />
        <parent link="r_uleg" />
        <child link="r_lleg" />
        <dynamics damping="0.1" friction="0" />
        <limit effort="220" velocity="12" lower="0" upper="2.45" />
        <safety_controller k_position="100" k_velocity="100" soft_lower_limit="-10" soft_upper_limit="12.45" />
    </joint>
</robot>
