# Dinamik Motor Eşleme v10

Pi artık fiziksel motor görevlerini YKİ'den alır:

```text
CMD:MAP:sol,ileri1,sag,ileri2
```

Her değer 1-4 arasında ve birbirinden farklı olmalıdır.

Varsayılan:

```text
CMD:MAP:1,2,3,4
```

Bu durumda:

- M1 sola yatay
- M2 ileri-1
- M3 sağa yatay
- M4 ileri-2

İleri motorların ikisi aynı `ileri_komutu` değişkeninden beslenir. Manuel ve otonom motor dağıtımı aynı dinamik eşlemeyi kullanır.
