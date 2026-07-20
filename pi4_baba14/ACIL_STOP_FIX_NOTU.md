# Acil STOP sonrası manuel moda dönememe düzeltmesi

Sorun STM tarafında değildir. STM geçerli sıfır PWM paketlerini alıp 0xAC ACK döndürmektedir.

Eski telemetri akışında `CMD:STOP`, PING bağlantısı kurulmamış olsa bile kabul ediliyordu.
Fakat `CMD:MOD:0` yalnızca aktif PING oturumunda kabul edildiğinden araç `AcilDurum`
modunda kalabiliyordu.

Bu sürümde geçerli checksum'lu `CMD:MOD:0` güvenli bir yeniden bağlantı/re-arm komutu
olarak kabul edilir. Bu komut tek başına motor hareketi oluşturmaz. Ardından `CMD:MAN`
komutu uygulanabilir. YKİ yine saniyede en az bir kez `CMD:PING*3C` göndermelidir;
PING gelmezse 4 saniyelik güvenlik watchdog'u bağlantıyı keser ve motorları durdurur.

Beklenen sıra:

1. `CMD:PING*3C\n`
2. `CMD:MOD:0*58\n`
3. `CMD:MAN:<ileri>,<yatay>*<CS>\n`
