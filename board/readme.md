Board notes
===========
R1 & R2 could be eliminated by using PWM on the LED output pins to limit the current. In this case, 
they should be repalced by a short. This would also require firmware changes.

D4 and R6 are optional. They are only used to implement communication though the power jack. 
If this communication channel is not needed, then D4 should be replaced by a short and R6 omitted. 
