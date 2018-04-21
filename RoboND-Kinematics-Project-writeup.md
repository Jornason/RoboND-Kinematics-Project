

# RoboND-Kinematics-Project-writeup

​    

### Kinematic Analysis

#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![QQ20180420-162747@2x](https://github.com/Jornason/RoboND-Kinematics-Project/blob/master/gif/2018-04-20-083001.jpg?raw=true)







![Canvas 1](http://owj75jsw8.bkt.clouddn.com/2018-04-20-092127.jpg)

alpha = arm twist angle

a = arm link length

d = arm link offset

theta = arm joint angle

| joint   | alpha | a      | d     | theta   |
| ------- | ----- | ------ | ----- | :------ |
| 1       | 0     | 0      | 0.75  | q1      |
| 2       | -p2/2 | 0.35   | 0     | q2-pi/2 |
| 3       | 0     | 1.25   | 0     | q3      |
| 4       | -pi/2 | -0.054 | 1.50  | q4      |
| 5       | pi/2  | 0      | 0     | q5      |
| 6       | -pi/2 | 0      | 0     | q6      |
| gripper | 0     | 0      | 0.303 | 0       |



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Standard Homogenous Transformation matrix from frame i-1 to frame i using DH Parameters:

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/gif.latex?T_%7Bi-1%7D%5E%7Bi%7D%3D%5Cbegin%7Bbmatrix%7Dcos%28%5Ctheta_%7Bi%7D%29%26-sin%28%5Ctheta_%7Bi%7D%29%260%26a_%7Bi-1%7D%20%5C%5C%20sin%28%5Ctheta_%7Bi%7D%29cos%28%5Calpha_%7Bi-1%7D%29%26cos%28%5Ctheta_%7Bi%7D%29cos%28%5Calpha_%7Bi-1%7D%29%26-sin%28%5Calpha_%7Bi-1%7D%29%26%20sin%28%5Calpha_%7Bi-1%7D%29d_%7Bi%7D%20%5C%5C%20sin%28%5Ctheta_%7Bi%7D%29sin%28%5Calpha_%7Bi-1%7D%29%26cos%28%5Ctheta_%7Bi%7D%29sin%28%5Calpha_%7Bi-1%7D%29%26cos%28%5Calpha_%7Bi-1%7D%29%26cos%28%5Calpha_%7Bi-1%7D%29d_%7Bi%7D%20%5C%5C%200%260%260%261%20%5C%5C%20%5Cend%7Bbmatrix%7D)



we get the following transformation matrices about each joint with respect to the previous joint:



![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/gif.latex?T_%7B0%7D%5E%7B1%7D%3D)

| cos(q1) | -sin(q1) | 0    | 0    |
| ------- | -------- | ---- | ---- |
| sin(q1) | cos(q1)  | 0    | 0    |
| 0       | 0        | 1    | 0.75 |
| 0       | 0        |      | 1    |

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?T_%7B1%7D%5E%7B2%7D%3D)

| sin(q2) | cos(q2)  | 0    | 0.35 |
| ------- | -------- | ---- | ---- |
| 0       | 0        | 1    | 0    |
| cos(q2) | -sin(q2) | 0    | 0    |
| 0       | 0        | 0    | 1    |

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?T_%7B2%7D%5E%7B3%7D%3D)

| cos(q3) | -sin(q3) | 0    | 1.25 |
| ------- | -------- | ---- | ---- |
| sin(q3) | cos(q3)  | 0    | 0    |
| 0       | 0        | 1    | 0    |
| 0       | 0        | 0    | 1    |


![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?T_%7B3%7D%5E%7B4%7D%3D)

| cos(q4)  | -sin(q4) | 0    | -0.054 |
| -------- | -------- | ---- | ------ |
| 0        | 0        | 1    | 1.5    |
| -sin(q4) | -cos(q4) | 0    | 0      |
| 0        | 0        | 0    | 1      |


![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?T_%7B4%7D%5E%7B5%7D%3D)

| cos(q5) | -sin(q5) | 0    | 0    |
| ------- | -------- | ---- | ---- |
| 0       | 0        | -1   | 0    |
| sin(q5) | cos(q5)  | 0    | 0    |
| 0       | 0        | 0    | 1    |


![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?T_%7B5%7D%5E%7B6%7D%3D)

| cos(q6)  | -sin(q6) | 0    | 0    |
| -------- | -------- | ---- | ---- |
| 0        | 0        | 1    | 0    |
| -sin(q6) | -cos(q6) | 0    | 0    |
| 0        | 0        | 0    | 1    |


![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?T_%7B6%7D%5E%7BG%7D%3D)

| 1    | 0    | 0    | 0     |
| ---- | ---- | ---- | ----- |
| 0    | 1    | 0    | 0     |
| 0    | 0    | 1    | 0.303 |
| 0    | 0    | 0    | 1     |

Generalized homogeneous transform:

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?T_%7B0%7D%5E%7BG%7D%3D%20T_0%5E1%20%5Ccdot%20T_1%5E2%20%5Ccdot%20T_2%5E3%20%5Ccdot%20T_3%5E4%20%5Ccdot%20T_4%5E5%20%5Ccdot%20T_5%5E6%20%5Ccdot%20T_6%5EG)



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.

[![alt text](https://github.com/udacity/RoboND-Kinematics-Project/raw/master/misc_images/misc3.png)](https://github.com/udacity/RoboND-Kinematics-Project/blob/master/misc_images/misc3.png)



**Rotation matrices**

ROLL

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?ROT_x%20%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%26%200%20%5C%5C%200%20%26%20cos%28r%29%20%26%20-sin%28r%29%20%5C%5C%200%20%26%20sin%28r%29%20%26%20cos%28r%29%20%5C%5C%20%5Cend%7Bbmatrix%7D)







PITCH  

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?ROT_y%20%3D%20%5Cbegin%7Bbmatrix%7D%20%28%20cos%28p%29%20%26%200%20%26%20sin%28p%29%20%5C%5C%200%20%26%201%20%26%200%20%5C%5C%20-sin%28p%29%20%26%200%20%26%20cos%28p%29%20%5C%5C%20%5Cend%7Bbmatrix%7D)



YAW

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?ROT_z%20%3D%20%5Cbegin%7Bbmatrix%7D%28cos%28y%29%20%26%20-sin%28y%29%20%26%200%20%5C%5C%20sin%28y%29%20%26%20cos%28y%29%20%26%200%20%5C%5C%200%20%26%200%20%26%201%20%5C%5C%20%5Cend%7Bbmatrix%7D)



![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?ROT_%7BEE%7D%20%3D%20ROT_z%20%5Ccdot%20ROT_y%20%5Ccdot%20ROT_x)



**$180^o$ clockwise rotation and $90^o$ clockwise rotation**

```python
Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
ROT_EE = ROT_EE * Rot_Error
```

Extract end-effector position and orientation from request. But since roll, pitch, and yaw values for the gripper are returned in quaternions, we can use the transformations.py module from the TF package.
Now that we can know WC.

```python
ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
EE = Matrix([[px], [py], [pz]])
WC = EE - (0.303) * ROT_EE[:,2]
```

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?%5Ctheta_1%3Datan2%28WC_y%2CWC_x%29)

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?%7B%5Ctheta_2%7D%3D%7B%5Cpi%5Cover2%7D%20-%20A%20-%20%5Batan2%28W%20%5Ccdot%20C_z%20-%200.75%2C%20%5Csqrt%7B%28W%20%5Ccdot%20C_x%5E2%20+%20W%20%5Ccdot%20C_y%5E2%20-%200.35%7D%29%5D)

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?%5Ctheta_3%20%3D%20%7B%5Cpi%5Cover%202%7D%20-%20%5BB%20+%200.036%5D)





Calculate joint angles using Geomatric IK method

```
side_b = sqrt(pow(sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35, 2)+ pow((WC[2] - 0.75), 2))
```

Inverse Orientation problems

![This is the rendered form of the equation. You can not edit this directly. Right click will give you the option to save the image, and in most browsers you can drag the image onto your desktop or another program.](https://latex.codecogs.com/png.latex?R%5E0_6%20%3D%20R%5E0_1%20%5Ccdot%20R%5E1_2%20%5Ccdot%20R%5E2_3%20%5Ccdot%20R%5E3_4%20%5Ccdot%20R%5E4_5%20%5Ccdot%20R%5E5_6)

Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of individual rotations between respective links, following holds true:

```
R0_6 = ROT_EE
```

We can substitute the values we calculated for joints 1 to 3 in their respective individual rotation matrices and pre-multiply both sides of the above equation by inv(R0_3) which leads to:

```
R3_6 = inv(R0_3) * ROT_EE
```

The matrix on the RHS (Right Hand Side of the equation) does not have any variables after substituting the joint angle values, and hence comparing LHS (Left Hand Side of the equation) with RHS will 
result in equations for joint 4, 5, and 6.

```
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})
R3_6 = R0_3.inv("LU") * ROT_EE

theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

Creates Homogeneous Transform Matrix from DH parameters

```python
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([
            [cos(q), 		-sin(q), 		0, 		a],
    		[sin(q)*cos(alpha), 	cos(q)*cos(alpha), 	-sin(alpha), 	-sin(alpha)*d],
    		[sin(q)* sin(alpha), 	cos(q)*sin(alpha), 	cos(alpha), 	cos(alpha)*d],
    		[0,			0,			0,		1]
    ])
    return TF
```

Define DH Parameters:

```python
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
```

Define DH Transformation Matrix

```python
DH_Table = {alpha0: 0, 	a0: 0, 		d1: 0.75, 	q1: q1,
            alpha1: -pi/2., a1: 0.35,	d2: 0, 		q2: -pi/2. + q2,
            alpha2: 0, 	a2: 1.25, 	d3: 0, 		q3: q3,
            alpha3: -pi/2., a3: -0.054, 	d4: 1.5, 	q4: q4,
            alpha4: pi/2, 	a4: 0, 		d5: 0, 		q5: q5,
            alpha5: -pi/2., a5: 0, 		d6: 0, 		q6: q6,
            alpha6: 0, 	a6: 0, 		d7: 0.303, 	q7: 0}
```

Generalized homogeneous transform

```python
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```





move to blue cylindrical target



​          ![2018-04-21-18-04-32.gif](https://github.com/Jornason/RoboND-Kinematics-Project/blob/master/gif/2018-04-21-18-04-32.gif?raw=true)      





pick the target

​          ![2018-04-21-18-04-32.gif](https://github.com/Jornason/RoboND-Kinematics-Project/blob/master/gif/2018-04-21-17-35-25.gif?raw=true)



move to drop location

​          ![2018-04-21-18-04-32.gif](https://github.com/Jornason/RoboND-Kinematics-Project/blob/master/gif/2018-04-21-17-35-52.gif?raw=true)





​          ![2018-04-21-18-04-32.gif](https://github.com/Jornason/RoboND-Kinematics-Project/blob/master/gif/2018-04-21-23-14-07.gif?raw=true)



release the target

​          ![2018-04-21-18-04-32.gif](https://github.com/Jornason/RoboND-Kinematics-Project/blob/master/gif/2018-04-21-22-58-06.gif?raw=true)



Shot of bin with 10 samples

![屏幕快照 2018-04-21 下午11.21.58](https://github.com/Jornason/RoboND-Kinematics-Project/blob/master/gif/2018-04-21-152448.jpg?raw=true)





This is my first time to use rviz and Gazebo, I found  they are extremely challenging to debug. My code may not be efficient enough, I can introduce LU decomposition in finding the inverse of matrix.







