#!/usr/bin/env python3
"""Pi koduna dokunmadan kare görevin beklenen zaman çizelgesini gösterir."""

KENAR_S = 10.0
DONUS_S = 2.5
ILERI_PWM = 400
DONUS_PWM = 400

zaman = 0.0
for kenar in range(1, 5):
    print(f"{zaman:5.1f}-{zaman + KENAR_S:5.1f} s | kenar {kenar}/4 ileri | M1=0 M2=0 M3={ILERI_PWM} M4={ILERI_PWM}")
    zaman += KENAR_S
    if kenar < 4:
        print(f"{zaman:5.1f}-{zaman + DONUS_S:5.1f} s | sola dönüş      | M1={DONUS_PWM} M2=0 M3=0 M4=0")
        zaman += DONUS_S
print(f"{zaman:5.1f} s sonrası          | görev tamam       | M1=0 M2=0 M3=0 M4=0")
