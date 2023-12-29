Video for 1st lab work: https://1drv.ms/v/s!Aifr0NSk-1EuhqMV1FIbZcwahpo5SA?e=764to5

Video for 2nd lab work: https://1drv.ms/v/s!Aifr0NSk-1EuhqMWtoyMGtUc8jf_Uw?e=aqZDAD

Переворачиваем матрицу и видим плату:

![image](https://github.com/tieensnh/Lab01/assets/115117887/42155261-3817-4021-b2d8-2637ac259979)

На плате кучка микросхем логики. Давайте разберёмся, что это за микросхемы:
1. 1 x SM74HC245D — неинвертирующий буфер
2. 1 x SM74HC04 — 6-канальный инвертор
3. 1 x SM74HC138D — 8-битный дешифратор
4. 4 x APM4953 — сборка из 2 P-канальных MOSFET
5. 16 x 74HC595D — сдвиговый регистр с защёлкой

![image](https://github.com/tieensnh/Lab01/assets/115117887/46613cf7-946c-49cc-af61-13bb9e43a720)

Принципиальная схема:

![image](https://github.com/tieensnh/Lab01/assets/115117887/ada059e4-8995-41d2-adeb-5d7bf776ef7a)
