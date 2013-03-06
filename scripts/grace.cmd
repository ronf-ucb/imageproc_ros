# http://plasma-gate.weizmann.ac.il/Xmgr/doc/commands.html
# commands to grace to plot telemetry data from ROS/optitrack
# run with xmgrace -batch grace.cmd -nxy Data/telemdata.dat
# ARRANGE(nrows, ncols, offset, hgap, vgap) 
arrange ( 3, 2, .20, .3, .30)
# ARRANGE(nrows, ncols, offset, hgap, vgap) 
# g0 g1
# g2 g3
# g4 g5
#######################
# graph 0 is gyro
focus g0
autoscale
yaxis label "gyro"
#xaxis label "time (sec)"
# change colors for gyro/accel
s4 line color 1
s4 line linewidth 2.0
s4 line linestyle 4
s4 legend "wx"
s5 line color 3
s5 line linewidth 2.0
s5 line linestyle 2
s5 legend "wy"
s6 line color 4
s6 line linewidth 2.0
s6 legend "wz"
kill g0.s7
# kill gyro avg
legend 0.5, 0.8
legend box false
legend char size 0.5
###############################
# graph 2 = acccelerometer
move g0.s8 to g2.s0
move g0.s9 to g2.s1
move g0.s10 to g2.s2
focus g2
s0 line color 1
s0 line linewidth 2.0
s0 line linestyle 4
s0 legend "ax"
s1 line color 3
s1 line linewidth 2.0
s1 line linestyle 2
s1 legend "ay"
s2 line color 4
s2 line linewidth 2.0
s2 legend "az"
legend 0.5, 0.6
legend box false
legend char size 0.5
autoscale
#xaxis label "time (sec)"
yaxis label "accel"
################################
# graph 4 = pwm values
move g0.s2 to g4.s0
move g0.s3 to g4.s1
focus g4
autoscale
yaxis tick major 2000
xaxis label "time (sec)"
yaxis label "motor PWM cmd"
# set line color and thickness
    s0 line linestyle 4
    s0 line linewidth 2.0
    s0 line color 4
    s0 legend "left"
    s1 line linestyle 1
    s1 line linewidth 2.0
    s1 line color 1
    s1 legend "right"
    legend 0.5, 0.37
legend char size 0.5
################################### 
# graph 1 motor position
move g0.s0 to g1.s0
move g0.s1 to g1.s1
focus g1
autoscale
#xaxis label "time (sec)"
yaxis label "leg phase (rad)"
#world ymax 12.6
# set line color and thickness
    s0 line linestyle 4
    s0 line linewidth 2.0
    s0 line color 4
    s0 legend "left"
    s1 line linestyle 1
    s1 line linewidth 2.0
    s1 line color 1
    s1 legend "right"
view ymin 0.63
    legend 0.98, 0.75
legend char size 0.5
#######################################
# graph 3 back emf values
move g0.s11 to g3.s0
move g0.s12 to g3.s1
focus g3
autoscale
# xaxis label "time (sec)"
yaxis  label "back emf"
# set line color and thickness
    s0 line linestyle 4
    s0 line linewidth 2.0
    s0 line color 4
    s0 legend "left"
    s1 line linestyle 1
    s1 line linewidth 2.0
    s1 line color 1
    s1 legend "right"
view ymin 0.41
view ymax 0.58
    legend 0.98, 0.47
legend char size 0.5
################################
# graph 5 Optitrack
# x,y,theta
move g0.s13 to g5.s0
move g0.s14 to g5.s1
move g0.s15 to g5.s2
# commanded linear and angular velocity
move g0.s16 to g5.s3
move g0.s17 to g5.s4
focus g5
xaxis label "time (sec)"
yaxis  label "optitrack (m,rad)"
    s0 line linestyle 4
    s0 line linewidth 2.0
    s0 line color 4
    s0 legend "x"
    s1 line linestyle 1
    s1 line linewidth 2.0
    s1 line color 1
    s1 legend "y"
    s2 line linestyle 2
    s2 line linewidth 2.0
    s2 line color 2
    s2 legend "theta"
    s3 legend "cmdV"
    s4 legend "cmd turn"
view ymax 0.37
legend 1.1, 0.37
legend char size 0.5
legend box off
autoscale
#yaxis tick major 2
#yaxis tick minor off
###################
# not sure what this stuff is for??
#legend
focus g0
    altxaxis  off
    altyaxis  off
    legend on
    legend loctype view

    legend box color 1
    legend box pattern 1
    legend box linewidth 1.0
    legend box linestyle 1
    legend box fill color 0
    legend box fill pattern 1
    legend font 0
    legend color 1
    legend length 4
    legend vgap 1
    legend hgap 1
    legend invert false
legend char size 0.5
    frame type 0
    frame linestyle 1
    frame linewidth 1.0
    frame color 1
    frame pattern 1
    frame background color 0
    frame background pattern 0


