### Description

```
btflash2 utility writes and/or verifies image into mitx boot flash.
The image must be exactly 16MB size. The tool is good for other
products too.

Usage:

# btflash2 <img_file> [-b] [-d] [-v] [-r] [-s<num>] [-f<num>]

Options:
	-b - batch mode (no progress is printed out)
	-d - verbose output
	-v - verify only (contents of flash is compared to the image)
	-r - read image into file
	-f <num> - start flashing from sector 'num'
	-s <num> - perform reads in small requests. 'num' must be power of 2
	   from 4 to 256. Default is 256.

> Работа с утилитой btflash2
> ==========================
>
> Если подгружен модуль mmc_spi, то он может мешать прошивке
> (создает большой количество прерываний в /proc/interrupts).
> Лучше его заранее выключить:  modprobe -r mmc_spi
>
> Можно предварительно сохранить старую прошивку:
> # btflash2 -r old_boot.rom
>
> Шить так:
> # ./btflash2 ./module.rom
>
> Иногда прошивка обламывается на каком-то секторе. В этом случае
> мы советуем запускать прошивку еще раз. Недавно Вадим Власов
> добавил возможность начинать шить не с нулевого сектора.
> Например, предыдущая прошивка остановилась с ошибкой:
> "Sector: 2398"
> Теперь можно продолжить шить прямо с этого места, либо немного до
> него:
> btflash2 -f 2397 ./mitx.rom
> 
> Таким образом, полная сессия прошивки может выглядеть так:
> 
> [Грузимся в обычный линукс]
> Затем:
> $ su
> # modprobe -r mmc_spi
> # ./btflash2 ./mitx.rom
> (тут происходит облом записи с ошибкамир, выдается сообщение об ошибке
> с указанием сектора)
> # ./btflash2 -f 2397 ./mitx.rom
> # ./btflash2 -r readin.rom
> # md5sum readin.rom
> # md5sum mitx.rom
```
