# Beyin UART v9 değişiklikleri

Motor eşlemesi:

- M1 (`iskeleon`) = sola yatay
- M2 (`iskelearka`) = ileri-1
- M3 (`sancakon`) = sağa yatay
- M4 (`sancakarka`) = ileri-2

Manuel paket biçimi:

```text
CMD:MAN:ileri,yatay
```

- `ileri`: `0.0..1.0`
- `yatay`: `-1.0..1.0`
- negatif yatay = sol
- pozitif yatay = sağ

Güvenlik:

- `0.02` ölü bölge uygulanır.
- M2 ve M4 her zaman aynı ileri komutunu alır.
- Manuel komut 500 ms boyunca yenilenmezse bütün motorlar sıfırlanır.
- Otonom sürüşte de M2 ve M4 eşit tutulur; yön düzeltmesi M1/M3 ile yapılır.
