#Kalau lupa buka ini

# Coordinate system 
x --> Dari robot ke net
y --> Simpangan/Kanan-kiri robot
Z --> Ketinggian Vertikal


# Event
from Solve_ivp
Event [ground,BackpostHit,Ringplate,Ring,MasukRing]





Code -1

# Collision

Code 0 --> Backpost Collision
Code 1 --> Ringplate Collision
Code 2 --> Ring collision
Code 3 --> Masuk Ring

# Out of Bound
Code 5 --> X out of Bound
Code 6 --> Ground Collision (Perlukah?) Akan ditambah di Z = 2.4?