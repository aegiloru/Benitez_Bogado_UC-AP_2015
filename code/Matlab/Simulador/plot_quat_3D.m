global A
[A.X22,A.Y22,A.Z22]=rotateQUAD(A.Body1X,A.Body1Y,A.Body1Z,A.q0,A.q1,A.q2,A.q3);
set(A.Body1,'xdata',A.X22,'ydata',A.Y22,'zdata',A.Z22+1)

[A.X22,A.Y22,A.Z22]=rotateQUAD(A.Body2X,A.Body2Y,A.Body2Z,A.q0,A.q1,A.q2,A.q3);
set(A.Body2,'xdata',A.X22,'ydata',A.Y22,'zdata',A.Z22+1)

[A.X22,A.Y22,A.Z22]=rotateQUAD(A.RotorFX,A.RotorFY,A.RotorFZ,A.q0,A.q1,A.q2,A.q3);
set(A.RotorF,'xdata',A.X22,'ydata',A.Y22,'zdata',A.Z22+1)

[A.X22,A.Y22,A.Z22]=rotateQUAD(A.RotorBX,A.RotorBY,A.RotorBZ,A.q0,A.q1,A.q2,A.q3);
set(A.RotorB,'xdata',A.X22,'ydata',A.Y22,'zdata',A.Z22+1)

[A.X22,A.Y22,A.Z22]=rotateQUAD(A.RotorLX,A.RotorLY,A.RotorLZ,A.q0,A.q1,A.q2,A.q3);
set(A.RotorL,'xdata',A.X22,'ydata',A.Y22,'zdata',A.Z22+1)

[A.X22,A.Y22,A.Z22]=rotateQUAD(A.RotorRX,A.RotorRY,A.RotorRZ,A.q0,A.q1,A.q2,A.q3);
set(A.RotorR,'xdata',A.X22,'ydata',A.Y22,'zdata',A.Z22+1)