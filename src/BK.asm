;	processor 16f876А
list p=16f876a, f=inhx8m

#include 	<p16f876a.inc>
#include 	<lcd_symbols.inc>
__config	0x3f72

           ;****************************
           ;* Раздел описания констант *
           ;****************************
;english lcd charset
#define LCD_ENGLISH

;use custom lcd characters
#define USE_CUSTOM_CHARS

;skip temperatures screen
#define SKIP_TEMP_SCREEN

; total distance (for eeprom saving)
#define TOTAL_KM	.297876

; default fuel constant
#define FUEL_CONSTANT	.109

; default vss constant
#define PULSES_PER_KM	.16000

; reset daily distance (km) (A)
#define MAX_COUNTER_A   .1000

; reset daily distance (km) (B)
#define MAX_COUNTER_B   .10000

; show average fuel consumption after total consumption of AVERAGE_MIN_FUEL * 0,01 litres
#define AVERAGE_MIN_FUEL .5

; show average speed (or fuel consumption) after distance AVERAGE_MIN_DIST * 0.1 km
#define AVERAGE_MIN_DIST .3

; taho round
#define TAHO_ROUND .10

; min speed for change idle screen
#define MIN_SPEED .5

; voltmeter constant
#define VOLT_CONSTANT .174

; use proteus simulator (no eeprom and fix for switch off injector)
;#define PROTEUS_SIM

; limit injector working time (proteus simulator)
#ifdef PROTEUS_SIM
    #define CHECK_INJ_TIME
#endif

; long key press time (* 0,1 s)
#define LONGKEY_TIME	.10

;lcd1 mode modes count
#define LCD1_MODE_COUNT .3
;lcd2 mode modes count
#define LCD2_MODE_COUNT .4
;lcdTO mode modes count
#define LCD_TO_MODE_COUNT .5
;lcd counter A/B mode modes count
#define LCD_AB_MODE_COUNT .2



;=================================================
; ОСНОВНЫЕ РЕГИСТРЫ
; записываются в ЕЕPROM

;variables	udata 0x20

;odo_01H		res 1
;odo_01F		res 1
;odo_01L		res 1

odo_01H		equ	h'20'	; основной счетчик пробега верхний байт
odo_01F		equ	h'21'	; основной счетчик пробега средний байт
odo_01L		equ	h'22'	; основной счетчик пробега нижний байт

odo_00H		equ	h'23'	; суточный счетчик пробега верхний байт
odo_00L		equ	h'24'	; суточный счетчик пробега нижний байт

odo_S1H		equ	h'25'	; сервисный счетчик пробега 1 верхний байт
odo_S1L		equ	h'26'	; сервисный счетчик пробега 1 нижний байт

odo_S2H		equ	h'27'	; сервисный счетчик пробега 2 верхний байт
;28
odo_S2L		equ	h'28'	; сервисный счетчик пробега 2 нижний байт

odo_S3H		equ	h'29'	; сервисный счетчик пробега 3 верхний байт
odo_S3L		equ	h'2a'	; сервисный счетчик пробега 3 нижний байт

odo_S4H		equ	h'2b'	; сервисный счетчик пробега 4 верхний байт
odo_S4L		equ	h'2c'	; сервисный счетчик пробега 4 нижний байт

FUEL_00H	equ	h'2d'	; счетчик расхода верхний байт
FUEL_00L	equ	h'2e'	; счетчик расхода нижний байт

odo_00Htemp	equ	h'2f'	; предварительный счетчик
odo_00Ltemp	equ	h'30'	; суточного пробега

FUEL_TMP2	equ	h'31'	; предварительный счетчик
FUEL_TMP1	equ	h'32'	; топлива

odo_01Htemp	equ	h'33'	; предварительный счетчик
odo_01Ltemp	equ	h'34'	; основного пробега

; константы
FUEL_CONST	equ	h'35'	; костанта для расчета расхода

ODO_CON1H	equ	h'36'	; костанта - число
ODO_CON1L	equ	h'37'	; импульсов на 1 км

;38
ZUMER_ON	equ	h'38'	; регистр флагов: озвучки событий, типы датчиков и т.д.
				; 0 - звук кнопок, 1 - звук температура ОЖ > 102
				; 2 - звук ТО по сервис-счетчикам, 3 -
				; 4 - , 5 - тип впрыска
				; 6 - тип экрана (0=ВЛИ, 1=ЖКИ), 7 - тип датчика температуры (0=B,1=S)
FUEL_MAX	equ	h'39'	; максимальный объем бака
fFLAGS_REG4	equ	h'3a'	; Регистр флагов.
;=================================================
; ОПЕРАТИВНЫЕ РЕГИСТРЫ

ODO_CON4H	equ	h'3d'	;  костанта - для расчета
ODO_CON4L	equ	h'3e'	;  мгновенного расхода

KMH_H		equ	h'3f'
;40
KMH_L		equ	h'40'
KMH_Htemp	equ	h'41'
KMH_Ltemp	equ	h'42'

FUEL_TMP3	equ	h'43'
FUEL_TMP4	equ	h'44'
TAHO_H		equ	h'45'
TAHO_L		equ	h'46'


;******** ДИСПЛЕЙ *************
LCD_CT1		equ	h'47'	; счетчик
;48
LCD_CT2		equ	h'48'	; счетчик
LCD_PLACE	equ	h'49'	; регистр с положением индицируемого символа
LCD_LCD		equ	h'4a'	; регистр с записыевым в LCD кодом
sec1		equ	h'4b'	;  счетчик нижний байт
sec2		equ	h'4c'	;  счетчик верхний байт

;******** ЧАСЫ  I2C *************
_i2c_byte	equ	h'4d'
_tmp		equ	h'4e'
EE_ADR		equ	h'4f'
;50
EE_DATA		equ	h'50'

; ***********
						;БИТЫ   | 8| 4| 2| 1| 8| 4| 2| 1|
MINUTES		equ	h'51'	;Минуты 		| 0| 10 MIN |  MINUTS   |  00-59
HOURS		equ	h'52'	;Часы	 	 	|12/24| -|10hr |  HOUR  |  00-23/1-12 ;BIT8 = 0-24 HOUR
DAYOFWEEK	equ	h'53'	;День недели		| 0| 0| 0| 0| 0| DAYOFW |  1-7
DATE		equ	h'54'	;День месяца		| 0| 0|10DAT|    DATE   |  1-31
MONTH		equ	h'55'	;МЕСЯЦ	 		| 0| 0| 0|10|   MONTH   |  1-12
YEAR		equ	h'56'	;ГОД   		     	| 10 YEAR   |    YEAR   |  00-99
tTIME		equ	h'57'

;58
; ******** ТЕРМОМЕТР 1-Ware *************
fTEMPER_L	equ     h'58'	; В эти регистры помещается температура после вызов
fTEMPER_H	equ     h'59'	; процедуры Get_Temperature.
fCNT_REM	equ     h'5a'
fCNT_PER_C	equ     h'5b'
fTRY		equ     h'5c'	; Отсчитывет попытки считывания температуры.
fCOUNTER	equ     h'5d'	; Счетчик (используется в процедур х delay и Big_delay).
fCOUNTER2	equ     h'5e'	; Счетчик2 (используется в процедуре Big_delay).
fBIT_CNT	equ     h'5f'	; Счетчик прочитанных битов.
;60
fTEMP		equ     h'60'	; Временный регистр только для локального использования.
fCRC		equ     h'61'	; Контрольная сумма.
fSAVE_W		equ	h'62'


; -------------------- Регистры для математики (процедуры mul_16, div_16 используют R0-R5).
R0		equ	h'63'
R1		equ	h'64'
R2		equ	h'65'
R3		equ	h'66'
R4		equ	h'67'
;68
R5		equ	h'68'
R6		equ	h'69'
R7		equ	h'6a'


; -------------------------------------------------------------------------------------
menu_position	equ	h'6b'	; регистр текущего номера меню
fFLAGS_REG1	equ	h'6c'	; Регистр флагов.
fFLAGS_REG2	equ	h'6d'	; Регистр флагов.
fFLAGS_REG3	equ	h'6e'	; Регистр флагов.
S_fCOUNTER	equ	h'6f'	; счетчик на таймер 2 сек.
;70
W_TEMP		equ	h'70'	; регистр сохранения содержимого W при прерываниях
ST_TEMP		equ	h'71'	; регистр сохранения содержимого STATUS при прерываниях
PCLATH_TEMP	equ	h'72'
fTIMER		equ	h'73'	; таймер интервала между измерениями

; -------------------- Регистры для работы зуммера ----------------------------
ZUM_tmp		equ	h'74'	; локальное использование в циклах
ZUM_N		equ	h'75'	; число повторов
ZUM_H		equ	h'76'	; длительность сигнала
ZUM_L		equ	h'77'	; длительность паузы

;78
;=====================================================================================

R8		equ     h'78'
R9		equ     h'79'
prev_menu	equ     h'7A'
key1_counter	equ     h'7B'
key2_counter	equ     h'7C'
R13		equ     h'7D'
R14		equ     h'7E'
R15		equ     h'7F'

;=====================================================================================

; ОЗУ БАНК 1
TEMPER1_L 	equ	h'A0'	; сдвоенный регистр температуры IN
TEMPER1_H 	equ	h'A1'	; 1-го датчика
TEMPER2_L  	equ	h'A2'	; сдвоенный регистр температуры OUT
TEMPER2_H  	equ	h'A3'	; 2-го датчика
TEMPER3_L  	equ	h'A4'	; сдвоенный регистр температуры ENGINE
TEMPER3_H  	equ	h'A5'	; 3-го датчика

; from eeprom 0x1A - ...
TO_1  		equ	h'A6'	; ТО масло двиг.
TO_2  		equ	h'A7'	; ТО масло АКПП
TO_3  		equ	h'A8'	; ТО свечи
TO_4  		equ	h'A9'	; ТО возд. фильтр

VCC_CONSTANT	equ	h'AA'	; константа напряжения
_DUMMY		equ	h'AB'	; резерв для константы

#ifdef CHECK_INJ_TIME
injOn		equ	h'AD'	; время работы форсунки (если больше 0,1с * 4 - отключаем tmr0)
#endif

; START EEPROM ADDR 0x48
time_H 		equ	0xB0	; время	движения 3 байта (2 сек), обнуляется вместе с суточным счетчиком
time_F		equ	0xB1
time_L 		equ	0xB2
mh_Htemp	equ     0xB3	; предварительный счетчик часов работы двигателя (2 сек), макс значение 3600/2 = 1800
mh_Ltemp	equ     0xB4
mh_H		equ     0xB5	; счетчик моточасов 2 байта
mh_L		equ     0xB6
mh_limit_H	equ     0xB7	; лимит моточасов 2 байта
mh_limit_L	equ     0xB8

odo_00H_B  	equ     0xB9	; суточный
odo_00L_B	equ     0xBA	; пробег (B)
odo_00Htemp_B	equ     0xBB	; пред. счетчик
odo_00Ltemp_B 	equ     0xBC	; суточного пр. (B)
FUEL_00H_B	equ     0xBD	; счетчик расхода
FUEL_00L_B 	equ     0xBE	; топлива	(B)
FUEL_TMP2_B	equ     0xBF	; предварительный
FUEL_TMP1_B 	equ     0xC0	; счетчик топлива (B)
time_H_B	equ     0xC1	; время	движения 3 байта (2 сек), обнуляется вместе с суточным счетчиком (B)
time_F_B	equ     0xC2
time_L_B 	equ     0xC3
MODE_LCD1	equ     0xC4	; показ на экране 1 в режиме дв/хх (0 - температура, 1 - напряжение, 2 - пробег текущей поездки)
MODE_LCD2	equ     0xC5	; показ на экране 2 (0 - общий расход, 1 - средняя скорость, 2 - время поездки, 3 - максимальная скорость)
odo_00H_C	equ	0xC6	; параметры пробега текущей поездки
odo_00L_C	equ	0xC7
odo_00Htemp_C	equ	0xC8
odo_00Ltemp_C	equ	0xC9
time_H_C	equ	0xCA	; время текущей поездки
time_F_C	equ	0xCB
time_L_C	equ	0xCC
FUEL_00H_C	equ	0xCD	; топливо текущей поездки
FUEL_00L_C	equ	0xCE
FUEL_TMP2_C	equ	0xCF
FUEL_TMP1_C	equ	0xD0
MINUTES_C	equ	0xD1	; дата завершения поездки
HOURS_C		equ	0xD2
DATE_C		equ	0xD3
MONTH_C		equ	0xD4
YEAR_C		equ	0xD5		
; ...
_DUMMY2		equ	0xD7
; END EEPROM ADDR 0x6F

	cblock	0xD8
; значение ADC для напряжения (текущее, минимальное, максимальное)
	volt:0,voltH,voltL
	volt_min:0,voltHmin,voltLmin
	volt_max:0,voltHmax,voltLmax
; значение скорости (текущее за 2 сек, максимальное за время работы)
	speed:0,speedH,speedL
	speed_max:0,speedHmax,speedLmax
	tVolt:0,tVoltHH,tVoltH,tVoltF,tVoltL
	tVoltCounter:0,tVoltCounterH,tVoltCounterL
	MODE_LCD_TO, MODE_LCD_A, MODE_LCD_B
	endc

; ОЗУ БАНК 2
EEADR_LAST	equ 0x120	; конечный адрес блока для чтения/записи EEPROM

	cblock 0x128 ; Адрес текущего датчика температуры
	fROM_ID:0,fROM_ID0,fROM_ID1,fROM_ID2,fROM_ID3,fROM_ID4,fROM_ID5,fROM_ID6,fROM_ID7
	endc

;=====================================================================================


           ;****************************
           ;* Раздел описания констант *
           ;****************************

;=====================================================================================
#define		CRC_OK      fFLAGS_REG1,0 	; Флаг правильности CRC
#define  	SEC2_OK     fFLAGS_REG1,1	; Флаг интервала 2 сек
#define  	ZERO	    fFLAGS_REG1,2	; Флаг гашения нулей
#define  	ODOM_FL     fFLAGS_REG1,3	; Флаг обработки импульса от датчика скорости
#define  	FUEL_FL     fFLAGS_REG1,4	; Флаг обработки импульса от форсунки
#define  	DRIVE_FL    fFLAGS_REG1,5	; Флаг наличия движения
#define  	MOTOR_FL    fFLAGS_REG1,6	; Флаг работы двигателя
#define  	ODOM_FL2_   fFLAGS_REG1,7	; Флаг 1 километра

#define  	button_OK1  fFLAGS_REG2,0	; Флаг обработки нажатия кнопки 1
#define  	button_FL1  fFLAGS_REG2,1	; Флаг действия кнопки 1
#define  	button_OK2  fFLAGS_REG2,2	; Флаг обработки нажатия кнопки 2
#define  	button_FL2  fFLAGS_REG2,3	; Флаг действия кнопки 2
#define  	TIME_FL	    fFLAGS_REG2,4	; Флаг коррекции времени
#define  	TAHO_FL	    fFLAGS_REG2,5	; Флаг измерения тахометра
#define  	TAHO_FL2    fFLAGS_REG2,6	; Флаг готовности тахометра
#define		fTIMER_FL   fFLAGS_REG2,7	; Флаг таймера измерения температуры

#define 	FL100	    fFLAGS_REG3,0	; Флаг готовности измерения разгона до 100
#define  	FL100_in    fFLAGS_REG3,1	; Флаг измерения разгона до 100
#define  	FL_zumer    fFLAGS_REG3,2	; Флаг вывода сигнала
#define  	FL_zumerS   fFLAGS_REG3,3	; Флаг работы зумера
#define  	FL_zumerH   fFLAGS_REG3,4	; Флаг вывода паузы зумера
#define  	FL_zumerВ1  fFLAGS_REG3,5	; Флаг зумер1
#define  	FL_zumerВ2  fFLAGS_REG3,6	; Флаг зумер2
#define  	FL_zumerВ3  fFLAGS_REG3,7	; Флаг зумер3

#define 	time_OK	    fFLAGS_REG4,0	; флаг лояльности часов
#define 	counterB    fFLAGS_REG4,1	; флаг счетчика B (0 - счетчик A, 1 - счетчик B)
#define		skipSpaces  fFLAGS_REG4,2	; выравнивание по левому краю (пропуск пробелов)
#define		button_OK1L fFLAGS_REG4,3	; длинное нажатие кнопки 1
#define		button_OK2L fFLAGS_REG4,4	; длинное нажатие кнопки 2
#define		timer01sec  fFLAGS_REG4,5	; флаг 0,1 сек (для расчета среднего напряжения за 0,1 сек)
#define		skipTemp    fFLAGS_REG4,6	; пропуск температурного экрана (при отсутствии датчиков)
#define 	counterC    fFLAGS_REG4,7	; флаг счетчика C (1 - счетчик C, 0 - счетчик A/B)

#define		ON_PWR	    PORTA,0 		; выход отключения питания
#define		CONTROL_PWR PORTA,1		; Вход контроля питания
#define		T_1WIRE	    PORTA,5   		; Выход на датчик температуры

#DEFINE		zumer	    PORTC,0
#DEFINE		LCD_RS_OUT  PORTC,1
#DEFINE		LCD_RW	    PORTC,2
#DEFINE		LCD_E	    PORTC,3

#define 	SDA	    PORTB,0
#define 	SCL	    PORTB,1
#define		button1	    PORTB,2		; вход на кнопку1
#define		button2	    PORTB,3		; вход на кнопку2
#define 	Tx	    PORTB,6		; вход датчика движения
#define 	FUEL	    PORTB,7		; вход датчика топлива

#define 	TRIS_SDA    TRISB^80,0 		; I2C SDA PIN
#define 	TRIS_SCL    TRISB^80,1 		; I2C SCL PIN

; max mode for screen 1 (0..MAX_MODE_LCD1)
MAX_MODE_LCD1	equ LCD1_MODE_COUNT - .1
; max mode for screen 2 (0..MAX_MODE_LCD2)
MAX_MODE_LCD2	equ LCD2_MODE_COUNT - .1
; max mode for screen TO
MAX_MODE_LCD_TO	equ LCD_TO_MODE_COUNT - .1
; max mode for screen A/B
MAX_MODE_LCD_AB	equ LCD_AB_MODE_COUNT - .1

;=====================================================================================

           ;****************************
           ;* Раздел  макроопределений *
           ;****************************

;=====================================================================================
JZ	macro	LBL		;Макрокоманда "Переход по условию,
	btfsc	STATUS,Z	;что результат предыдущей операции
	goto	LBL		;был нулевым".
	endm

;====================================================================================
JZN	macro	LBL		;Макрокоманда "Переход по условию,
	btfss	STATUS,Z	;что результат предыдущей операции
	goto	LBL		;был ненулевым".
	endm

;====================================================================================
prv_w	macro	REG
	movf	REG,w
	andlw	b'00001111'
	iorlw	b'00110000'
	movwf	LCD_LCD
	endm
;====================================================================================
prv_wi	macro	REG
	swapf	REG,w
	andlw	b'00001111'
	iorlw	b'00110000'
	movwf	LCD_LCD
	endm
;====================================================================================
BANK0   macro
	BCF	STATUS,RP1
	BCF	STATUS,RP0
	errorlevel	-302
	errorlevel	-306
	endm
;====================================================================================
BANK1   macro
	BCF	STATUS,RP1
	BSF	STATUS,RP0
	errorlevel	-302
	endm
;====================================================================================
BANK2   macro
	BSF	STATUS,RP1
	BCF	STATUS,RP0
	endm
;====================================================================================
BANK3   macro
	BSF	STATUS,RP1
	BSF	STATUS,RP0
	endm
;====================================================================================
callp	macro	addr
	pageselw	addr
	call		addr
	endm
;====================================================================================
gotop	macro	addr
	pageselw	addr
	goto		addr
	endm
;====================================================================================
MSLOVO_XXX macro start, len
	movlw	start
	movwf	_tmp
	movlw	len
	movwf	_i2c_byte
	callp	SLOVO_XXX
	endm

MSLOVO_YYY macro start, len
	movlw	start
	movwf	_tmp
	movlw	len
	movwf	_i2c_byte
	callp	SLOVO_YYY
	endm
;====================================================================================

MOVR7R0R1R2	macro num
	movlw	((num) >> .24) & 0xFF
	movwf	R7
	movlw	((num) >> .16) & 0xFF
	movwf	R0
	movlw	((num) >>  .8) & 0xFF
	movwf	R1
	movlw	((num) >>  .0) & 0xFF
	movwf	R2
	endm

MOVR0R1R2	macro num
	movlw	((num) >> .16) & 0xFF
	movwf	R0
	movlw	((num) >>  .8) & 0xFF
	movwf	R1
	movlw	((num) >>  .0) & 0xFF
	movwf	R2
	endm

;	загрузка в FSR адреса регистра в зависимости от счетчика
FSR_ABC	macro regA,regB,regC
	movlw	regA
	btfsc	counterB
	movlw	regB
	btfsc	counterC
	movlw	regC
	movwf	FSR
	endm

inc16 macro regH,regL
	incf	regL
	btfsc	STATUS,Z
	incf	regH
	endm

inc16w macro reg
	incf	reg + 0x01
	btfsc	STATUS,Z
	incf	reg + 0x00
	endm

; НАЧАЛО ПРОГРАММЫ.
	org   0 		; Адрес начала  Flash.
 	clrf  PCLATH        	; ensure page bits are cleared
   	goto  start     	; go to beginning of program

	org	4		; вектор прерывания

;====================================================================================
; ОБРАБОТКА ПРЕРЫВАНИЯ
;====================================================================================
;------сохранение регистров W, STATUS----------------------
	movwf	W_TEMP
	swapf	STATUS,W
	clrf	STATUS
	movwf	ST_TEMP
	movf	PCLATH,W
	movwf	PCLATH_TEMP
	clrf	PCLATH
;====================================================================================
; прерывание от RB6...7
	btfsc	INTCON,RBIE
	btfss	INTCON,RBIF
	goto	PR_T0

	movf	PORTB,w
	bcf	INTCON,RBIF	; сбросим флаг прерывания от RB6..7

;================================
;	обработка датчика топлива
;	активный фронт 1 -> 0
	btfsc	FUEL		; смотрим состояние входа форсунки
	goto	PR_B10		; форсунка выключена
	btfsc	FUEL_FL
	goto	PR_B20		; флаг установлен, форсунка открыта, знначит
				; прерывание от датчика скорости
				; иначе запускаем счетчик открытия форсунки

	BANK1
	bcf	OPTION_REG,5	; включаем счетчик TMR0
#ifdef CHECK_INJ_TIME
	clrf	injOn
#endif
	BANK0

	bsf	FUEL_FL		; устанавливаем флаг отсчета интервала
	bsf	MOTOR_FL	; установим флаг работающего двигателя

;__________________________ УПРАВЛЕНИЕ ТАЙМЕРОМ TMR2
	btfss	TAHO_FL2
	;goto	PR_T0
	goto	PR_B20
	btfss	TAHO_FL
	goto	PR_B9

	bcf	T2CON,TMR2ON	; выкл. отсчет интервала
	bcf	TAHO_FL
	bcf	TAHO_FL2
	;goto	PR_T0
	goto	PR_B20

PR_B9
	clrf	TMR2
	bcf	PIR1,TMR2IF
	bsf	T2CON,TMR2ON	; вкл. отсчет интервала
	bsf	TAHO_FL
	;goto	PR_T0
	goto	PR_B20

PR_B10
	btfss	FUEL_FL
	goto	PR_B20		; флаг сброшен, выходим
	BANK1		        ; выбор банка 1
	bsf	OPTION_REG,5	; выключаем счетчик TMR0
	BANK0		        ; выбор банка 0
	bcf	FUEL_FL		; сбрасываем флаг отсчета интервала
	;goto	PR_T0
	goto	PR_B20

;================================
;	обработка датчика скорости
;	активный фронт 0 -> 1
PR_B20
	btfss	Tx		; смотрим состояние входа датчика скорости
	goto	PR_B1		; если изменение в 0, то просто сбросим флаг ODOM_FL

	btfsc	ODOM_FL		; иначе если изменение в 1, смотрим флаг ODOM_FL
	goto	PR_T0		; если 1, то ничего не делаем

	bsf	DRIVE_FL	; установим флаг наличия движения
	bsf	ODOM_FL		;

	incf	KMH_Ltemp,f	; регистры
	btfsc	STATUS,Z	; для расчета
	incf	KMH_Htemp,f	; скорости

;__________________________ измерение скорости по длительности импульса VSS
	btfss	FL100		; режим замера разгона до 100
	goto	PR_B21
	btfss	FL100_in
	goto	PR_B25

	bcf	T2CON,TMR2ON	; выкл. отсчет интервала
	bcf	FL100
	bcf	FL100_in
	goto	PR_B21

PR_B25
	clrf	TMR2
	bcf	PIR1,TMR2IF
	bsf	T2CON,TMR2ON	; вкл. отсчет интервала
	bsf	FL100_in

PR_B21
	; обработка основного одометра
	; increment temp odo_01
	incf	odo_01Ltemp
	btfsc	STATUS,Z
	incf	odo_01Htemp

	movf	ODO_CON1H, w		;  Compare the High Byte First
	subwf	odo_01Htemp, w
	btfss	STATUS, Z		;  If Result of High Byte Compare
	goto	_odo01_end1		;  is Equal to Zero, Then Check
	movf	ODO_CON1L, w		;  the Second
	subwf	odo_01Ltemp, w
_odo01_end1
	btfss	STATUS,C		;  Carry Flag Set if 1st >= 2nd
	goto	_odo01_end

;odo_01Htemp:odo_01Ltemp >= ODO_CON1H:ODO_CON1L
	clrf	odo_01Htemp
	clrf	odo_01Ltemp

; приращение счетчиков
	call	ODOM_INC

_odo01_end

	; счетчик суточного пробега А
	; increment temp odo_00
	incf	odo_00Ltemp
	btfsc	STATUS,Z
	incf	odo_00Htemp

	movf	ODO_CON1H, w		;  Compare the High Byte First
	subwf	odo_00Htemp, w
	btfss	STATUS, Z		;  If Result of High Byte Compare
	goto	_odo00_end1		;  is Equal to Zero, Then Check
	movf	ODO_CON1L, w		;  the Second
	subwf	odo_00Ltemp, w
_odo00_end1
	btfss	STATUS,C		;  Carry Flag Set if 1st >= 2nd
	goto	_odo00_end

;odo_00Htemp:odo_00Ltemp >= ODO_CON1H:ODO_CON1L
	clrf	odo_00Htemp
	clrf	odo_00Ltemp

	; increment odo_00
	incf	odo_00L
	btfsc	STATUS,Z
	incf	odo_00H
	
_odo00_end

	; счетчик суточного пробега B
	; increment temp odo_00_B
	BANK1
	incf	odo_00Ltemp_B
	btfsc	STATUS,Z
	incf	odo_00Htemp_B

	BANK0
	movf	ODO_CON1H, w		;  Compare the High Byte First
	BANK1
	subwf	odo_00Htemp_B, w
	btfss	STATUS, Z		;  If Result of High Byte Compare
	goto	_odo00_B_end1		;  is Equal to Zero, Then Check
	BANK0
	movf	ODO_CON1L, w		;  the Second
	BANK1
	subwf	odo_00Ltemp_B, w
_odo00_B_end1
	btfss	STATUS,C		;  Carry Flag Set if 1st >= 2nd
	goto	_odo00_B_end

;odo_01Htemp_B:odo_01Ltemp_B >= ODO_CON1H:ODO_CON1L
	clrf	odo_00Htemp_B
	clrf	odo_00Ltemp_B

	; increment odo_00_B
	incf	odo_00L_B
	btfsc	STATUS,Z
	incf	odo_00H_B

_odo00_B_end
	
	; счетчик пробега текущей поездки C
	; increment temp odo_00_C
	incf	odo_00Ltemp_C
	btfsc	STATUS,Z
	incf	odo_00Htemp_C

	BANK0
	movf	ODO_CON1H, w		;  Compare the High Byte First
	BANK1
	subwf	odo_00Htemp_C, w
	btfss	STATUS, Z		;  If Result of High Byte Compare
	goto	_odo00_C_end1		;  is Equal to Zero, Then Check
	BANK0
	movf	ODO_CON1L, w		;  the Second
	BANK1
	subwf	odo_00Ltemp_C, w
_odo00_C_end1
	btfss	STATUS,C		;  Carry Flag Set if 1st >= 2nd
	goto	_odo00_C_end

;odo_01Htemp_C:odo_01Ltemp_C >= ODO_CON1H:ODO_CON1L
	clrf	odo_00Htemp_C
	clrf	odo_00Ltemp_C

	; increment odo_00_B
	incf	odo_00L_C
	btfsc	STATUS,Z
	incf	odo_00H_C

_odo00_C_end
	BANK0

	goto	PR_T0

PR_B1
	bcf	ODOM_FL

;====================================================================================
; прерывание от TMR0
; таймер используется для измерения расхода
; топлива
PR_T0
	btfsc	INTCON,T0IE
	btfss	INTCON,T0IF
	goto	PR_T1

	incf	sec1,f
	btfsc	STATUS,Z
	incf	sec2,f

; счетчик топлива A
	decf	FUEL_TMP1,f
	btfss	STATUS,Z
	goto	PR_T01
	movlw	.65			; первый счетчик считает до 65
	btfsc	ZUMER_ON,5
	movlw	.130			; для парного впрыска больше в 2 раза
	movwf	FUEL_TMP1
	incf	FUEL_TMP2,f		; второй счетчик считает до константы (190-210)
	movf	FUEL_CONST,w
	subwf	FUEL_TMP2,w
	JZN	PR_T01
	clrf	FUEL_TMP2
	incf	FUEL_00L,f		; инкримент равен 10 мл
	btfsc	STATUS,Z
	incf	FUEL_00H,f

PR_T01

	BANK1
; счетчик топлива B
	decf	FUEL_TMP1_B,f
	btfss	STATUS,Z
	goto	PR_T02
	BANK0
	movlw	.65			; первый счетчик считает до 65
	btfsc	ZUMER_ON,5
	movlw	.130			; для парного впрыска больше в 2 раза
	BANK1
	movwf	FUEL_TMP1_B
	incf	FUEL_TMP2_B,f		; второй счетчик считает до константы (190-210)
	BANK0
	movf	FUEL_CONST,w
	BANK1
	subwf	FUEL_TMP2_B,w
	JZN	PR_T02
	clrf	FUEL_TMP2_B
	incf	FUEL_00L_B,f		; инкримент равен 10 мл
	btfsc	STATUS,Z
	incf	FUEL_00H_B,f

PR_T02

; счетчик топлива C
	decf	FUEL_TMP1_C,f
	btfss	STATUS,Z
	goto	PR_T03
	BANK0
	movlw	.65			; первый счетчик считает до 65
	btfsc	ZUMER_ON,5
	movlw	.130			; для парного впрыска больше в 2 раза
	BANK1
	movwf	FUEL_TMP1_C
	incf	FUEL_TMP2_C,f		; второй счетчик считает до константы (190-210)
	BANK0
	movf	FUEL_CONST,w
	BANK1
	subwf	FUEL_TMP2_C,w
	JZN	PR_T03
	clrf	FUEL_TMP2_C
	incf	FUEL_00L_C,f		; инкримент равен 10 мл
	btfsc	STATUS,Z
	incf	FUEL_00H_C,f

PR_T03
	BANK0
	bcf	INTCON,T0IF

;====================================================================================
; прерывание от TMR1
; таймер используется для подсчета временных интервалов измерений
; 1. Таймер - 2 сек
; 2. Таймер замера времени разгона до 100
; 3. Таймер 0,1 сек лдя работы зумера
; переполнение через 100мс
PR_T1
	btfss	PIR1,TMR1IF
	goto	PR_T2
	bcf	PIR1,TMR1IF
	movlw	h'0B'
	movwf	TMR1H
	movlw	h'DC'
	movwf	TMR1L

	btfss	FL_zumer
	goto	PR_T11
	gotop	ZUMER_		; обработчик вывода сигналов

PR_T11
	movlw	menu_100	; обработка таймера
	subwf	menu_position,w	; в режиме замера
	btfss	STATUS,Z	; разгона до 100
	goto	PR_T12
	btfsc	DRIVE_FL
	incf	fCRC
	decf	fTEMP,f

PR_T12
	bsf	timer01sec

;	счетчики нажатий клавиш
	btfss	button_FL1
	goto	_end_key1_counter
	movlw	LONGKEY_TIME + .1
	subwf	key1_counter,w
	btfss	STATUS,C
	incf	key1_counter
_end_key1_counter

	btfss	button_FL2
	goto	_end_key2_counter
	movlw	LONGKEY_TIME + .1
	subwf	key2_counter,w
	btfss	STATUS,C
	incf	key2_counter
_end_key2_counter

#ifdef CHECK_INJ_TIME
;	проверка времени работы форсунки
	BANK1
	btfsc	OPTION_REG,5
	goto	_skip_injtime_check
	incf	injOn
	movf	injOn,w
	xorlw	.2		; время работы счетчика топлива (* 0,1 c)
	btfsc	STATUS,Z
	bsf	OPTION_REG,5	; отключение счетчика TMR0
_skip_injtime_check
	BANK0
#endif

	decfsz	S_fCOUNTER,f
	goto	PR_T2
	movlw	.20
	movwf	S_fCOUNTER
	bsf	SEC2_OK

;	единица измерения временных счетчиков - 2 секунды
;	если установлен либо флаг движения, либо флаг работы мотора (т.к. в режиме принудительного х/х форсунки не работают)
	btfss	MOTOR_FL
	btfsc	DRIVE_FL
	goto	_inc_time
	goto	PR_T12_1

_inc_time
;	счетчик для расчета средней скорости
	BANK1
	incf	time_L
	btfsc	STATUS,Z
	incf	time_F
	btfsc	STATUS,Z
	incf	time_H

	incf	time_L_B
	btfsc	STATUS,Z
	incf	time_F_B
	btfsc	STATUS,Z
	incf	time_H_B

	incf	time_L_C
	btfsc	STATUS,Z
	incf	time_F_C
	btfsc	STATUS,Z
	incf	time_H_C

;	счетчик моточасов
	incf	mh_Ltemp
	btfsc	STATUS,Z
	incf	mh_Htemp

; 	максимальное значение 3600/2
	movlw	low .1800
	subwf   mh_Ltemp,w

	movlw   high .1800
	btfss   STATUS,C
	addlw   .1
	subwf   mh_Htemp,w

	btfss   STATUS,C
	goto    _mh_end

	clrf	mh_Ltemp
	clrf	mh_Htemp

	incf	mh_L
	btfsc	STATUS,Z
	incf	mh_H

_mh_end
	BANK0

PR_T12_1
	movf	KMH_Ltemp,w
	movwf	KMH_L
	movf	KMH_Htemp,w
	movwf	KMH_H
	clrf	KMH_Ltemp
	clrf	KMH_Htemp

	movf	sec1,w
	movwf	FUEL_TMP3
	movf	sec2,w
	movwf	FUEL_TMP4
	clrf	sec1
	clrf	sec2

	decfsz 	fTIMER,f
	goto	PR_T2
	movlw	.15
	movwf	fTIMER
	bsf	fTIMER_FL

;====================================================================================
; прерывание от TMR2
; таймер используется для измерения итервалов между вх. импульсами
; 1. измерения частоты вращения двигателя, вход FUEL
; 2. измерения скорости, вход Тх
; переполнение через 10мкс
; результат в рег. TAHO_L,TAHO_H
PR_T2
	btfss	PIR1,TMR2IF
	goto	PR_end
	bcf	PIR1,TMR2IF

	incf	TAHO_L,f
	btfsc	STATUS,Z
	incf	TAHO_H,f

; если счетчики переполнились принудительно остановим таймер
	btfss	STATUS,Z
	goto	PR_end
	bcf	T2CON,TMR2ON	; выкл. отсчет интервала
	movlw	0xff
	movwf	TAHO_L
	movwf	TAHO_H

	BANK1		        ;выбор банка 1
	bsf	OPTION_REG,5	; выключаем счетчик TMR0
	BANK0		        ;выбор банка 0

;-------Завершение обработки прерывания-----------------------

PR_end
	movf	PCLATH_TEMP,W
	movwf	PCLATH
	swapf	ST_TEMP,W
	movwf	STATUS
	swapf	W_TEMP,F
	swapf	W_TEMP,W
	retfie

;***********************************************************************************
            ;****************************
            ;*   Раздел инициализации   *
            ;****************************

	org	0x168
MENU_SELECT
	bcf	SEC2_OK
	movf	menu_position,W
	addwf	PCL,f

menu_item 	macro item
item		equ _menu_position
_menu_position	set _menu_position + 1
		endm

_menu_position	set	0

	menu_item   menu_main		    ; основной режим
	goto	LCD_MODE1

	menu_item   menu_daily		    ; пробег, расход
	goto	LCD_MODE2

#ifndef	SKIP_TEMP_SCREEN
	menu_item   menu_temp		    ; температура
	goto	LCD_MODE3
#endif

	menu_item   menu_counter_A	    ; пробег А
	goto	LCD_MODE18_A

	menu_item   menu_counter_B	    ; пробег B
	goto	LCD_MODE18_B

	menu_item   menu_voltage	    ; ср. скор., вольтметр (тек., мин., макс.)
	goto	LCD_MODE17

	menu_item   menu_clock		    ; часы расширенно
	goto	LCD_MODE4

	menu_item   menu_service	    ; сервисный счетчик моточасы
	goto	LCD_MODE7_1

max_menu_motor	equ _menu_position
max_menu    equ _menu_position

	menu_item   menu_daily_reset	    ; подтверждение сброса /menu_daily/
	goto	LCD_MODE5

	menu_item   menu_voltage_reset	    ; подтверждение сброса /voltage/
	goto	LCD_MODE17_reset

	menu_item   menu_service_reset	    ; подтверждение сброса /menu_service4/
	goto	LCD_MODE_RESET_TO

	menu_item   menu_100		    ; разгон до 100
	goto	LCD_MODE16

	menu_item   menu_100_confirm	    ; разгон до 100 /подтверждение/
	goto	LCD_MODE15

	menu_item   menu_clock_edit	    ; подтверждение коррекции часов /menu_clock/
	goto	LCD_MODE6


start:
	movlw	0x10
	movwf	PCLATH
	call	INIT		; установка портов, регистров
	clrf	PCLATH

	btfsc	button1
	goto	_start1
	call	tm40000		; задержка от дребезга контактов
	btfss	button1
	goto	srv100_		; вход в сервисный режим

_start1
	call	ALL_TEMP	; считывание данных с температурных датчиков
	callp	_ADC_VOLT	; измерение напряжения
	pageselw $
	goto	LCD_MODE1

srv100_
	callp	LCD_CLEAR
	movlw 	0x7F
	movwf	LCD_PLACE
	MSLOVO_XXX	0x41, 0x0B	; СЕРВИС МЕНЮ
	pageselw $
srv100_1
	btfss	button1
	goto	srv100_1
	gotop	srv100
;====================================================================================
          ;****************************
          ;*  Главный цикл программы  *
          ;****************************

;=====================================================================================
main:
	btfsc	fTIMER_FL
	call	ALL_TEMP 	; считывание температуры через 30 сек.

	callp	ADC_VOLT	; измерение напряжения

	pageselw $
	call	scan		; сканирование клавиатуры
	call 	SCANP		; проверка зажигания

	btfsc	button_OK1
	btfsc	TIME_FL
	goto	main1

	movf	prev_menu,W
	btfsc	STATUS,Z
	goto	main12
	movwf	menu_position
	goto	main11
main12
	movlw	max_menu_motor
	incf	menu_position,F
	btfsc	MOTOR_FL
	goto	_max_menu_compare
	movlw	max_menu
_max_menu_compare
	subwf	menu_position,W
	btfsc	STATUS,C
	clrf	menu_position
main11
	clrf	prev_menu
	bcf	button_OK1
	bcf	button_OK2
	bcf	button_OK1L
	bcf	button_OK2L

main1

	call	CLEAR_A_B

main3
	btfss	SEC2_OK		; если флаг установлен
	goto	main		; обновляем экран через 2 сек

	btfsc	DRIVE_FL
	call	CALC_SPEED	; расчет максимальной скорости

	gotop	MENU_SELECT

main2
	bcf	button_OK1
	bcf	button_OK2

	bcf	DRIVE_FL
	bcf	MOTOR_FL

	goto 	main

;==============================================================================================
;==============================================================================================
;==============================================================================================
            ;****************************
            ;*   Раздел подпрограмм     *
            ;****************************
ODOM_INC:
	incf	odo_01L,f
	btfsc	STATUS,Z
	incf	odo_01F,f
	btfsc	STATUS,Z
	incf	odo_01H,f

	incf	odo_S1L,f
	btfsc	STATUS,Z
	incf	odo_S1H,f

	incf	odo_S2L,f
	btfsc	STATUS,Z
	incf	odo_S2H,f

	incf	odo_S3L,f
	btfsc	STATUS,Z
	incf	odo_S3H,f

	incf	odo_S4L,f
	btfsc	STATUS,Z
	incf	odo_S4H,f
	return

CLEAR_A_B:
    
#ifdef MAX_COUNTER_A
	; zero odo_00 when greater than MAX_COUNTER_A
	movlw	low MAX_COUNTER_A
	subwf   odo_00L,w

	movlw   high MAX_COUNTER_A
	btfss   STATUS,C
	addlw   .1
	subwf   odo_00H,w

	btfss   STATUS,C
	goto    _clear_a_end

	callp	_CLEAR
	clrf	PCLATH
_clear_a_end
#endif

#ifdef MAX_COUNTER_B
	BANK1
	; zero odo_00_B when greater than MAX_COUNTER_B
	movlw	low MAX_COUNTER_B
	subwf   odo_00L_B,w

	movlw   high MAX_COUNTER_B
	btfss   STATUS,C
	addlw   .1
	subwf   odo_00H_B,w

	btfss   STATUS,C
	goto    _clear_b_end

	callp	_CLEAR_B
	clrf	PCLATH
	
_clear_b_end	
	BANK0
#endif
	return

CALC_SPEED
	bsf	PCLATH,3	; страница памяти 1

	movlw	high .18000
	movwf	R4
	movlw	low  .18000
	movwf	R3

	movf	KMH_L,w
	movwf	R5
	movf	KMH_H,w
	movwf	R6
	call	MUL16x16	; R4 : R3 * R6 : R5 -> R7:R0:R1:R2

	movf	ODO_CON1H,w
	movwf	R5
	movf	ODO_CON1L,w
	movwf	R4

	call	div32_16	; R1:R2 = R7:R0:R1:R2 / R5:R4 (R7:R0 - остаток)

	bcf	PCLATH,3	; страница памяти 0

;	move R1:R2 to speedH:speedL
	movlw	speedH
	movwf	FSR
	movf	R1,w
	movwf	INDF
	incf	FSR,f
	movf	R2,w
	movwf	INDF

	BANK1
	movf	speedLmax,w
	subwf	speedL,w
	movf	speedHmax,w
	btfss   STATUS,C
	addlw   .1
	subwf   speedH,w

	btfss   STATUS,C		;skip if speed >= speed_max
	goto    _speed_max_end		;goto if speed <  speed_max
	movf	speedL,w
	movwf	speedLmax
	movf	speedH,w
	movwf	speedHmax
_speed_max_end
	BANK0
	return


;=================================================================================================================
;  СКАНИРОВАНИЕ КЛАВИАТУРЫ
;=================================================================================================================
scan
	btfsc	button1
	goto	scan1
	btfss	button_FL1
	goto	_scan12
; проверка на длительное нажатие
	movlw	LONGKEY_TIME
	subwf	key1_counter,w
	btfss	STATUS,Z
	goto	_scan14
	bsf	button_OK1L
	movlw	LONGKEY_TIME + .1
	movwf	key1_counter
	btfss	ZUMER_ON,0
	goto	_scan14
	bsf	FL_zumer
	bsf	FL_zumerВ1
_scan14
	return
_scan12
	call	tm40000		; задержка от дребезга контактов
	btfsc	button1
	return
; кнопка нажата
	bsf	button_FL1
	clrf	key1_counter
	goto 	_scan1_end
scan1
; кнопка отпущена
	btfss	button_FL1
	goto	_scan13
; пропуск, если было длительное нажатие
	movlw	LONGKEY_TIME
	subwf	key1_counter,w
	btfsc	STATUS,C
	goto	_scan13
	bsf	button_OK1
	call	LCD_CLEAR 	; затираем экран
	bsf	SEC2_OK
_scan15
	btfss	ZUMER_ON,0
	goto	_scan13
	bsf	FL_zumer
	bsf	FL_zumerВ1
_scan13
	bcf	button_FL1
_scan1_end

	btfsc	button2
	goto	scan2
	btfss	button_FL2
	goto	_scan22
; проверка на длительное нажатие
	movlw	LONGKEY_TIME
	subwf	key2_counter,w
	btfss	STATUS,Z
	goto	_scan24
	bsf	button_OK2L
	movlw	LONGKEY_TIME + .1
	movwf	key2_counter
	btfss	ZUMER_ON,0
	goto	_scan24
	bsf	FL_zumer
	bsf	FL_zumerВ1
_scan24
	return
_scan22
	call	tm40000		; задержка от дребезга контактов
	btfsc	button2
	return
; кнопка нажата
	bsf	button_FL2
	clrf	key2_counter
	goto 	_scan2_end
scan2
; кнопка отпущена
	btfss	button_FL2
	goto	_scan23
; пропуск, если было длительное нажатие
	movlw	LONGKEY_TIME
	subwf	key2_counter,w
	btfsc	STATUS,C
	goto	_scan23
	bsf	button_OK2
	call	LCD_CLEAR 	; затираем экран
	bsf	SEC2_OK
_scan25
	btfss	ZUMER_ON,0
	goto	_scan23
	bsf	FL_zumer
	bsf	FL_zumerВ1
_scan23
	bcf	button_FL2
_scan2_end

	return


;==============================================================================================
; ПОДПРОГРАММА ЗАВЕРШЕНИЯ РАБОТЫ ПО ПРОПАДАНИЮ ПИТАНИЯ
;==============================================================================================
PWROFF
	call	tm40000
	call	tm40000
	btfsc	CONTROL_PWR		; еще раз проверяем питание
	return

; задержка в 1 сек. чтоб двигатель надежно заглох, напряжение устаканилось
;	movlw	.10
;	movwf	S_fCOUNTER
;	bcf	SEC2_OK
;	btfss	SEC2_OK
;	goto 	$-1

;	сохранение даты завершения поездки

	bsf	PCLATH,3
	call	TIME_R	    ; чтение текущего времени
	bcf	PCLATH,3
	
	movlw	MINUTES_C
	movwf	FSR
	
	movf	MINUTES,w
	movwf	INDF
	incf	FSR,f
	movf	HOURS,w
	movwf	INDF
	incf	FSR,f
	movf	DATE,w
	movwf	INDF
	incf	FSR,f
	movf	MONTH,w
	movwf	INDF
	incf	FSR,f
	movf	YEAR,w
	movwf	INDF
	
	callp	SAVE			;вызываем подпрограмму записи переменных

	pageselw $

	BANK0

   	bcf	ON_PWR			;выключаем питание схемы
	;btfss	CONTROL_PWR
   	;goto 	$-1			;зацикливаемся
	goto 	$

;==============================================================================================
SCANP
	btfss	CONTROL_PWR	; Проверяем напряжение питания
	call	PWROFF 		; выключаем питание
	return			; если есть то возвращаемся


;==============================================================================================
;==============================================================================================
;	ПРОЦЕДУРЫ РАБОТЫ С ЖКИ
;==============================================================================================
;==============================================================================================

; очистка мин/макс значения напряжений + макс. скорости
volt_clear
	BANK1
	clrf	voltHmax
	clrf	voltLmax
	movlw	0x7F
	movwf	voltHmin
	clrf	voltLmin
	decf	voltLmin,f

	clrf	speedHmax
	clrf	speedLmax

	BANK0

	callp	_ADC_VOLT
	pageselw $

	movlw	menu_voltage
	movwf	menu_position
	bcf	button_OK2
	bsf	SEC2_OK
	goto	main3

; очистка счетчиков A/B/C
ODOM_CLEAR
	btfss	counterC
	goto	_odom_clear1
	callp	_CLEAR_C
	clrf	PCLATH
	goto	_odom_clear2
_odom_clear1
	btfss	counterB
	goto	_odom_clear11
	callp	_CLEAR_B
	clrf	PCLATH
	goto	_odom_clear2
_odom_clear11
	callp	_CLEAR
	clrf	PCLATH
_odom_clear2
	;movlw	menu_daily
	movf	prev_menu,w			;old menu position
	movwf	menu_position
CLEAR_END
	bcf	button_OK2
	bsf	SEC2_OK
	goto	main3

; очистка счетчиков ТО
to_clear
	pageselw $
	BANK1
	movf	MODE_LCD_TO,w
	BANK0
	addwf	PCL,f
	goto	ODOM_CLEAR1
	goto	ODOM_CLEAR1
	goto	ODOM_CLEAR2
	goto	ODOM_CLEAR3
	goto	ODOM_CLEAR4

ODOM_CLEAR1
	clrf	odo_S1L
	clrf	odo_S1H

	BANK1
	clrf	mh_H
	clrf	mh_L
	clrf	mh_Htemp
	clrf	mh_Ltemp
	BANK0

	movlw	0x20		;Заносим в W адрес ячейки
	goto	ODOM_TO_CLEAR

ODOM_CLEAR2
	clrf	odo_S2L
	clrf	odo_S2H
	movlw	0x23		;Заносим в W адрес ячейки
	goto	ODOM_TO_CLEAR

ODOM_CLEAR3
	clrf	odo_S3L
	clrf	odo_S3H
	movlw	0x26		;Заносим в W адрес ячейки
	goto	ODOM_TO_CLEAR

ODOM_CLEAR4
	clrf	odo_S4L
	clrf	odo_S4H
	movlw	0x29		;Заносим в W адрес ячейки

ODOM_TO_CLEAR
	bsf	PCLATH,4	; страница памяти 1
	call	SAVE_DATE
	movlw	menu_service
	movwf	menu_position
	goto	CLEAR_END

CORR_TIME
;	переход в режим коррекции времени
; 	признак коррекции флаг TIME_FL,
; 	tTIME - указывет на то, что редактируем (часы, минуты, годы...)
	movlw	menu_clock
	movwf	menu_position
	bcf	button_OK2
	bsf	SEC2_OK
	btfss	time_OK	; проверим лояльность показаний
	goto	main3
	bsf	TIME_FL
	incf	tTIME,f

;	включаем курсор
	MOVLW	b'00001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11111111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200		;пауза 200 мкс
	goto	main3

LCD_MODE1
; первый экран
; 1) на месте с заглушенным двигателем
; время			общий пробег (км)
; нар.темп.		вольтметр
; 2) на месте с работающим двигателем
; время			тахометр (об/мин)
; нар.темп./вольтметр	средний расход (л/ч)
; 3) в движении
; скорость (км/ч)	тахометр (об/мин)
; нар.темп./вольтметр	средний расход (л/100км)

;	кнопка 2 - режимы показа на экране 1
	btfss	button_OK2
	goto	_skip_button_OK2
	bcf	button_OK2
	BANK1
	incf	MODE_LCD1,f
	movf	MODE_LCD1,w
	xorlw	MAX_MODE_LCD1 + .1
	btfsc	STATUS,Z
	clrf	MODE_LCD1
	BANK0
_skip_button_OK2

	btfss	DRIVE_FL
	goto	_no_drive_fl

; check if speed more than (MIN_SPEED) km/h
if MIN_SPEED > 0
	BANK1
	movf	speedH,w
	BANK0
	btfss	STATUS,Z
	goto	m1
	movlw	(MIN_SPEED * .10) - .1
	BANK1
	subwf	speedL,w
	BANK0
	btfss	STATUS,C
	goto	m2
endif
	goto	m1
_no_drive_fl
	btfsc	MOTOR_FL
	goto	m2

;	call	LCD_CLEAR

	bsf	PCLATH,3	; страница памяти 1
	movlw	0x88
	call	LCD_ODOM2_MAIN

	bsf	PCLATH,3	; страница памяти 1
	movlw	0xC9
	call	LCD_VOLT
	goto	m3

; ---------------режим движение------------
m1
;	call	LCD_CLEAR

#ifdef USE_CUSTOM_CHARS
	movlw	0x84
	movwf	LCD_PLACE
	movlw	_SPACE
	movwf	LCD_LCD
	call	LCD
#endif
 	bsf	PCLATH,3	; страница памяти 1
	movlw	0x80
	call	LCD_KMH_LA	; скорость

 	bsf	PCLATH,3	; страница памяти 1
	movlw	0xC9
	call	LCD_FUEL2	; расход на 100 км

	goto	m3_1

; ---------------режим ХХ------------
m2
	btfsc	button_OK2L	;вход в режим разгон до 100
	goto	mode1_lb

;	call	LCD_CLEAR
#ifdef USE_CUSTOM_CHARS
	movlw	0x88
	movwf	LCD_PLACE
	movlw	_SPACE
	movwf	LCD_LCD
	call	LCD
	call	LCD_INCPLACE
	movlw	0x8A
#else
	movlw	0x88
#endif

	bsf	PCLATH,3	; страница памяти 1
	call	LCD_TAHO	; тахометр

	bsf	PCLATH,3	; страница памяти 1
	movlw	0xC9
	call	LCD_FUEL3	; часовой расход

; --------------- время / температура ------------
m3	bsf	PCLATH,3	; страница памяти 1
	call	TIME_R
	movlw	0x7F
	call	LCD_TIME

	incf	LCD_PLACE
	call	LCD_SPACE
	call	LCD_SPACE
	call	LCD_SPACE

	btfsc	MOTOR_FL
	goto	m3_3

	goto	m3_2

m3_1
;	call	LCD_CLEAR

#ifdef USE_CUSTOM_CHARS
	movlw	0x88
	movwf	LCD_PLACE
	movlw	_SPACE
	movwf	LCD_LCD
	call	LCD
	call	LCD_INCPLACE
	movlw	0x8A
#else
	movlw	0x88
#endif
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_TAHO	; тахометр

m3_3
	pageselw $
	BANK1
	movf	MODE_LCD1,W
	BANK0
	addwf	PCL,f
	goto	m3_21
	goto	m3_2
	goto	m3_22	

m3_22
;	пробег текущей поездки
	bsf	counterC
	movlw	0xC0
	bsf	PCLATH,3	; страница памяти 1
	call 	LCD_ODOM
	goto	_m3_2

m3_21
;	вольтметр
	clrf	PCLATH
	bsf	PCLATH,3	; страница памяти 1
	movlw	0xBE
	call	LCD_VOLT

	incf	LCD_PLACE
	call	LCD_SPACE
	call	LCD_SPACE
	call	LCD_SPACE

	goto	_m3_2
m3_2
; температура за бортом
	clrf	PCLATH
	movlw	TEMPER2_L
	movwf	FSR
	call	READ_T
	movlw	0xC0
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_TEMP
_m3_2
	bcf	button_OK2L
	goto	main2

mode1_lb
	movlw	menu_100_confirm
	movwf	menu_position
	bcf	button_OK2L
	bsf	SEC2_OK
	call	LCD_CLEAR
	movlw	.30
	movwf	PCLATH		; страница памяти 3
	call	SLOVO8
	goto	main3

LCD_MODE15
	btfss	button_OK2
	goto	main2
	movlw	menu_100
	movwf	menu_position
	bcf	button_OK2
	bsf	SEC2_OK
	goto	main3

;=====================================================================================
; режим измерения разгона до 100 км/ч
;=====================================================================================
LCD_MODE16
	bsf	PCLATH,3	; страница памяти 1
	goto	SPEED100


; второй экран
; 1) на месте с заглушенным двигателем
; общий пробег		средний расход (л/100км)
; сут. пробег A (км)	общий расход (л)
; 2) на месте с работающим двигателем
; тахометр (об/мин)	средний расход (л/100км)
; сут. пробег A (км)	общий расход (л)
; 3) в движении
; тахометр (об/мин)	средний расход (л/100км)
; сут. пробег A (км)	общий расход (л)
;
LCD_MODE2
;	call	LCD_CLEAR
	bcf	counterC
	bcf	counterB	; счетчик A
	
	bsf	counterC	   

;	кнопка 2 - режимы показа на экране 2
	btfss	button_OK2
	goto	_skip_button_OK2_2
	bcf	button_OK2
	BANK1
	incf	MODE_LCD2,f
	movf	MODE_LCD2,w
	xorlw	MAX_MODE_LCD2 + .1
	btfsc	STATUS,Z
	clrf	MODE_LCD2
	BANK0
_skip_button_OK2_2

	btfss	button_OK2L	    ;сброс счетчика А
	goto	_skip_long_button22
	bcf	button_OK2L
mode2_1b
	movf	menu_position,w
	movwf	prev_menu	    ; previous menu position
	movlw	menu_daily_reset
	movwf	menu_position
	bsf	SEC2_OK
	call	LCD_CLEAR
	goto	main3

_skip_long_button22

	btfsc	DRIVE_FL
	goto	mm2
	btfsc	MOTOR_FL
	goto	mm2_1

;	на месте двигатель заглушен
	movlw	0x80
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_ODOM2_MAIN
	;call	LCD_AVERAGE_SPEED
	goto	mm3
mm2_1
;	на месте двигатель работает
;	movlw	0x80
;	bsf	PCLATH,3	; страница памяти 1
;	call	LCD_TAHO
;	goto	mm3
mm2
;	в движении
	movlw	0x80
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_TAHO		; тахометр
	;call LCD_AVERAGE_SPEED

mm3
	movlw	0x89
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_FUEL4

	movlw	0xC0
	bsf	PCLATH,3	; страница памяти 1
	call 	LCD_ODOM

m3_select
	pageselw $
	BANK1
	movf	MODE_LCD2,W
	BANK0
	addwf	PCL,f
	goto	m3_31
	goto	m3_32
	goto	m3_33
	goto	m3_34
m3_31
	movlw	0xCA
	bsf	PCLATH,3	    ; страница памяти 1
	call 	LCD_FUEL
	goto	main2

m3_32
#ifdef USE_CUSTOM_CHARS
	movlw	0xCA
#else
	movlw	0xC8
#endif
	;bcf	PCLATH,3	    ; страница памяти 1
	call	LCD_AVERAGE_SPEED
	goto	main2

m3_33
	movlw	0xCA
	bsf	PCLATH,3	    ; страница памяти 1
	call 	LCD_TIME_COUNTER    ; время в движении
	goto	main2
m3_34
	pageselw LCD_KMH_MAX
	movlw	0xCA
	call	LCD_KMH_MAX	    ; максимальная скорость

	movlw	0xCA - .1
	movwf	LCD_PLACE
	callp	SLOVO29		    ;max
	goto	main2

LCD_MODE5					; сброс счетчиков A/B
	btfsc	button_OK2
	goto	ODOM_CLEAR
	goto	_slovo1

LCD_MODE17_reset
	btfsc	button_OK2
	goto	volt_clear
	goto	_slovo1

LCD_MODE_RESET_TO
	btfsc	button_OK2
	goto	to_clear
	goto	_slovo1
_slovo1
	callp	SLOVO1	; сброс?
	goto	main2

LCD_MODE7_1
	;	кнопка 2 - режимы показа на экране ТО
	btfss	button_OK2
	goto	_skip_button_OK2_27
	bcf	button_OK2
	BANK1
	incf	MODE_LCD_TO,f
	movf	MODE_LCD_TO,w
	xorlw	MAX_MODE_LCD_TO + .1
	btfsc	STATUS,Z
	clrf	MODE_LCD_TO
	BANK0
_skip_button_OK2_27

	btfss	button_OK2L	    ;сброс счетчика А
	goto	_skip_long_button27
	bcf	button_OK2L

lcd_mode7_long_button
	movf	menu_position,w
	movwf	prev_menu	    ; previous menu position
	movlw	menu_service_reset
	movwf	menu_position
	bsf	SEC2_OK
	call	LCD_CLEAR
	goto	main3

_skip_long_button27
	pageselw $
	BANK1
	movf	MODE_LCD_TO,w
	BANK0
	addwf	PCL,f
	goto	TO_MODE1
	goto	TO_MODE2
	goto	TO_MODE3
	goto	TO_MODE4
	goto	TO_MODE5

; экран счетчиков A/B
LCD_MODE18_A
	bcf	counterB
	movlw	MODE_LCD_A
	goto	_lcd_mode_18_main
LCD_MODE18_B
	bsf	counterB
	movlw	MODE_LCD_B
_lcd_mode_18_main
	bcf	counterC
	;	кнопка 2 - режимы показа на экране A/B
	movwf	FSR
	btfss	button_OK2
	goto	_lcd_mode_18_skip_button_OK2
	bcf	button_OK2
	incf	INDF,f
	movf	INDF,w
	xorlw	MAX_MODE_LCD_AB + .1
	btfsc	STATUS,Z
	clrf	INDF
_lcd_mode_18_skip_button_OK2

	btfss	button_OK2L	    ;сброс счетчиков А/B
	goto	_lcd_mode_18_skip_long_button
	bcf	button_OK2L

	movf	menu_position,w
	movwf	prev_menu	    ; previous menu position
	movlw	menu_daily_reset
	movwf	menu_position
	bsf	SEC2_OK
	call	LCD_CLEAR
	goto	main3

_lcd_mode_18_skip_long_button
;	слово пробег A/B
	movlw	0x7F
	movwf	LCD_PLACE
	callp	SLOVO30
	pageselw $
	movlw	_SPACE
	call	LCD_
	movlw	_A
	btfsc	counterB
	movlw	_B
	call	LCD_

	movlw	0x88
	bsf	PCLATH,3		; страница памяти 1
	call 	LCD_ODOM_NO_SKIP	; пробег

	pageselw $
	btfsc	counterB
	goto	select_part1_B
	BANK1
	movf	MODE_LCD_A,W
	goto	_select_part
select_part1_B
	BANK1
	movf	MODE_LCD_B,W
_select_part
	BANK0
	addwf	PCL,f
	goto	lcd_mode18_part1
	goto	lcd_mode18_part2

lcd_mode18_part1
; средний расход - средняя скорость
	movlw	0xC0 - .1
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_FUEL4	; средний расход
#ifdef USE_CUSTOM_CHARS
	movlw	0xCA
#else
	movlw	0xC8
#endif
	call	LCD_AVERAGE_SPEED   ;средняя скорость
	goto	main2
lcd_mode18_part2
; время - общий расход
	movlw	0xC0
	bsf	PCLATH,3	    ; страница памяти 1
	call 	LCD_TIME_COUNTER_LA ; время в движении

	movlw	0xCA
	bsf	PCLATH,3	    ; страница памяти 1
	call 	LCD_FUEL	    ; общий расход
lcd_mode18_end
	goto	main2
	
TO_MODE1
	movlw	.31
	movwf	PCLATH		; страница памяти 3
	call	SLOVO3_1
	BANK1
	movf	mh_L,w
	BANK0
	movwf	R3
	BANK1
	movf	mh_H,w
	BANK0
	movwf	R4
	clrf	R5

	movlw	0xC0
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_MH

	movlw	0x20
	goto	TO_MODE_TIME
TO_MODE2
	movlw	.31
	movwf	PCLATH		; страница памяти 3
	call	SLOVO3
	call	SLOVO3_ENGINE
	movlw	odo_S1H
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_ODOM2_TO
	movlw	0x20
	goto	TO_MODE_TIME
TO_MODE3
	movlw	.31
	movwf	PCLATH		; страница памяти 3
	call	SLOVO3
	call	SLOVO3_AKPP
	movlw	odo_S2H
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_ODOM2_TO
	movlw	0x23
	goto	TO_MODE_TIME
TO_MODE4
	movlw	.31
	movwf	PCLATH		; страница памяти 3
	call	SLOVO4
	movlw	odo_S3H
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_ODOM2_TO
	movlw	0x26
	goto	TO_MODE_TIME

TO_MODE5
	movlw	.31
	movwf	PCLATH		; страница памяти 3
	call	SLOVO5
	movlw	odo_S4H
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_ODOM2_TO
	movlw	0x29
TO_MODE_TIME
	call	LCD_TIME3
	goto	main2


LCD_MODE3 ; температура
;	call	LCD_CLEAR
	btfsc	skipTemp	; пропуск экрана темепратур при отсутствии датчиков
	goto	_skip_lcd_mode3
	bsf	PCLATH,3	; страница памяти 1
	movlw	0x89
	call	LCD_VOLT

	movlw	.31
	movwf	PCLATH		; страница памяти 3
	movlw	0x7f
	call	SLOVO6		; IN
	movlw	TEMPER1_L
	movwf	FSR
	call	READ_T
	movlw	0x84
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_TEMP

	movlw	.31
	movwf	PCLATH		; страница памяти 3
	movlw	0xbf
	call	SLOVO7		; OUT
	movlw	TEMPER2_L
	movwf	FSR
	call	READ_T
	movlw	0xC4
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_TEMP

	movlw	.30
	movwf	PCLATH		; страница памяти 3
	movlw	0xCA
	call 	SLOVO10		; ОЖ
	movlw	TEMPER3_L
	movwf	FSR
	call	READ_T
	movlw	0xCC
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_TEMP2
	goto	main2
_skip_lcd_mode3
	bsf	SEC2_OK
	goto	main12

LCD_MODE17 ; макс. скорость, напряжение (тек., мин., макс.)
;	call	LCD_CLEAR

	btfsc	button_OK2
	goto	mode17_lb

	pageselw LCD_KMH_MAX
	movlw	0x80
	call	LCD_KMH_MAX	; максимальная скорость

	movlw	0x80 - .1
	movwf	LCD_PLACE
	callp	SLOVO29		;max

	pageselw LCD_VOLT
	movlw	0x89
	call	LCD_VOLT	; текущее напряжение

	pageselw LCD_VOLT_MIN
	movlw	0xBF
	call	LCD_VOLT_MIN	;Umin value

	movlw 	0xBF
	movwf	LCD_PLACE
	callp	SLOVO28		;min

	pageselw LCD_VOLT_MAX
	movlw	0xC9
	call	LCD_VOLT_MAX	;Umax value

	movlw 	0xC9
	movwf	LCD_PLACE
	callp	SLOVO29		;max

	goto	main2

mode17_lb
;	call	LCD_CLEAR
	movf	menu_position,w
	movwf	prev_menu		; previous menu position
	movlw	menu_voltage_reset
	movwf	menu_position
	bcf	button_OK2
	bsf	SEC2_OK
	goto	main3

LCD_MODE4
;	call	LCD_CLEAR

	btfsc	TIME_FL
	goto	MODE41
	btfsc	button_OK2
	goto	mode4_lb
	bsf	PCLATH,3    ; страница памяти 1
	call	TIME_R	    ; чтение данных из таймера
	bcf	PCLATH,3    ; страница памяти 0

MODE41
	movlw	0x7F
	btfsc	time_OK
	goto	$+3
	call	LCD_TIME
	goto 	$+2
	call	LCD_TIME2
	goto	main2


mode4_lb
	movlw	menu_clock_edit
	movwf	menu_position
	bcf	button_OK2
	bsf	SEC2_OK
	goto	main3


LCD_MODE6
	btfsc	button_OK2
	goto	CORR_TIME
	movlw	.30
	movwf	PCLATH		; страница памяти 3
	call	SLOVO2
	goto	main2

;==============================================================================================
LCD_TIME2
	btfss	button_OK1
	goto	TIME22
	bcf	button_OK1
	incf	tTIME,f
	movf	tTIME,w
	xorlw	.7
	btfss	STATUS,Z
	goto	TIME21
	clrf	tTIME
	bcf	TIME_FL

	MOVLW	b'00001111'	; выклычаем курсоры
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT

	bsf	PCLATH,3	; страница памяти 1
	call	TIME_W
	bcf	PCLATH,3	; страница памяти 0
	goto	TIME21

TIME22
	btfsc	TIME_FL
	btfss	button_OK2
	goto	TIME21
	callp	TIME_
	clrf	PCLATH		; страница памяти 0

TIME21
	movlw 	0x80
	movwf	LCD_PLACE

	swapf	HOURS,w
	andlw	b'00000011'
	iorlw	b'00110000'
	movwf	LCD_LCD
	call 	LCD		; десятки часов

	prv_w	HOURS
	call 	LCD_INCPLACE	; единицы часов

	movlw	0x3A
	call 	LCD_		; двоеточие

	prv_wi MINUTES
	call 	LCD_INCPLACE	; десятки минут

	prv_w	MINUTES
	call 	LCD_INCPLACE	; единицы минут


	movlw	0x86
	movwf	LCD_PLACE

	prv_wi DATE
	call 	LCD_INCPLACE	; десятки дней

	prv_w	DATE
	call 	LCD_INCPLACE	; единицы дней

	callp	MONTH_SELECT

time11
#ifdef	LCD_ENGLISH
	movlw	0x27
#else
	movlw	0xE7
#endif
	btfss	ZUMER_ON,6
	movlw	0x27
	call 	LCD_		; знак '

	prv_wi YEAR
	call 	LCD_INCPLACE	; десятки год

	prv_w	YEAR
	call 	LCD_INCPLACE	; единицы год

	movlw	0xBF
	movwf	LCD_PLACE

	callp	DAY_SELECT

time12
	movf	tTIME,w
	btfsc	STATUS,Z
	return

; установка курсора в нужную позицию
KUR
	callp	KURSOR_		; выбор места курсора
	clrf	PCLATH		; страница памяти 0
	movwf	LCD_PLACE

	bcf	LCD_RS_OUT
	movlw	b'11110000'
	iorwf	PORTC,f	;установим все биты в "1", операция ИЛИ
	movf	LCD_PLACE,W
	iorlw	b'00001111'	; нижний полубайт в "1"
	andwf	PORTC,f
	BSF	LCD_E
	CALL	tm50		;пауза 50 мкс
	BCF	LCD_E

	movlw	b'11110000'
	iorwf	PORTC,f	;установим все биты в "1", операция ИЛИ
	swapf	LCD_PLACE,W
	iorlw	b'00001111'	; нижний полубайт в "1"
	andwf	PORTC,f
	BSF	LCD_E
	CALL	tm50		;пауза 50 мкс
	bcf	LCD_E

	return

;==============================================================================================
LCD_TIME3
	BANK2
	movwf	EEADR
	call 	EEREAD			;Типовая процедура чтения EEPROM
	movwf	R1
	call 	EEREAD			;Типовая процедура чтения EEPROM
	movwf	R2
	call 	EEREAD			;Типовая процедура чтения EEPROM
	movwf	R3

	movlw 0xC8
	movwf	LCD_PLACE

	comf	R1,w
	JZN	TIME31
	movlw	0x2D
	movwf	LCD_LCD
	call 	LCD			; -
	call 	LCD_INCPLACE	; -
	incf	LCD_LCD,f
	call 	LCD_INCPLACE	; .
	decf	LCD_LCD,f
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	incf	LCD_LCD,f
	call 	LCD_INCPLACE	; .
	decf	LCD_LCD,f
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	return

TIME31
	prv_wi	R1
	call 	LCD		; десятки дней
	prv_w	R1
	call 	LCD_INCPLACE	; единицы дней

	movlw	_POINT		;запятая
	call 	LCD_

	prv_wi 	R2
	call 	LCD_INCPLACE	; десятки месяц
	prv_w	R2
	call 	LCD_INCPLACE	; единицы месяц

	movlw	_POINT		;запятая
	call 	LCD_

	prv_wi 	R3
	call 	LCD_INCPLACE	; десятки год
	prv_w	R3
	call 	LCD_INCPLACE	; единицы год
	return

;=====================================================================================
; гашение лишних нулей слева
ZERO_	btfss	ZERO
	goto	ZERO_1
	movlw	b'00110000'
	xorwf	LCD_LCD,w	; проверка на 0
	JZN	ZERO_2		; если не равно 0, переход по ссылке
	goto	LCD_SPACE
	goto	ZERO_1
ZERO_2	bcf	ZERO
ZERO_1	call 	LCD
	incf	LCD_PLACE,f
	return

;=====================================================================================
; затирание ячейки экрана со сдвигом адреса
LCD_SPACE
	movlw _SPACE
	movwf LCD_LCD
	call LCD
	incf LCD_PLACE,f
	return

;=====================================================================================
; подпрограмма перевода R0 в двоично-десятичный код R0
BinBCD
	clrf     R0
again
	addlw    -.10
	btfss   STATUS,C
	goto    swapBCD
	incf    R0
	goto    again
swapBCD
	addlw   .10
	swapf    R0
	iorwf    R0
	return

;=====================================================================================
; подпрограмма перевода 3х байтных чисел R5:R4:R3 в двоично-десятичный код R0 R1 R2
B2_BCD bcf     STATUS,0                ; clear the carry bit
	movlw   .24
	movwf   R6
	clrf    R0
	clrf    R1
	clrf    R2
loop162 rlf     R3, F
	rlf     R4, F
	rlf     R5, F
	rlf     R2, F
	rlf     R1, F
	rlf     R0, F

	decfsz  R6, F
	goto    adjDEC
	RETLW   0

adjDEC	movlw   R2
	movwf   FSR
	call    adjBCD

	movlw   R1
	movwf   FSR
	call    adjBCD

	movlw   R0
	movwf   FSR
	call    adjBCD

	goto    loop162

adjBCD movlw   3
	addwf   0,W
	movwf   R7
	btfsc   R7,3          ; test if result > 7
	movwf   0
	movlw   30
	addwf   0,W
	movwf   R7
	btfsc   R7,7          ; test if result > 7
	movwf   0               ; save as MSD
	RETLW   0

;=====================================================================================
send_lcd_command ; LCD_PLACE
	BCF	LCD_RS_OUT
; установка адреса
	movlw	b'11110000'
	iorwf	PORTC,f	;установим все биты в "1", операция ИЛИ
	movf	LCD_PLACE,W
	iorlw	b'00001111'	; нижний полубайт в "1"
	andwf	PORTC,f
	BSF	LCD_E
	CALL	tm50		;пауза 50 мкс
	BCF	LCD_E

	movlw	b'11110000'
	iorwf	PORTC,f	;установим все биты в "1", операция ИЛИ
	swapf	LCD_PLACE,W
	iorlw	b'00001111'	; нижний полубайт в "1"
	andwf	PORTC,f
	BSF	LCD_E
	CALL	tm50		;пауза 50 мкс
	BCF	LCD_E
	return

send_lcd_data	; LCD_LCD
; установка данных
	CALL	tm100		;пауза 100мкс
	BSF	LCD_RS_OUT
	movlw	b'11110000'
	iorwf	PORTC,f	;установим все биты в "1", операция ИЛИ
	MOVF	LCD_LCD,W
	iorlw	b'00001111'	; нижний полубайт в "1"
	andwf	PORTC,f
	BSF	LCD_E
	CALL	tm50		;пауза 50 мкс
	BCF	LCD_E

	movlw	b'11110000'
	iorwf	PORTC,f	;установим все биты в "1", операция ИЛИ
	swapf	LCD_LCD,W
	iorlw	b'00001111'	; нижний полубайт в "1"
	andwf	PORTC,f
	BSF	LCD_E
	CALL	tm50		;пауза 50 мкс
	BCF	LCD_E

	BCF	LCD_RS_OUT
	movlw	b'11110000'
	iorwf	PORTC,f	;установим все биты в "1", операция ИЛИ
	return

LCD__
	movwf	LCD_LCD
	goto	LCD
LCD_
	movwf	LCD_LCD
	incf	LCD_PLACE,F
	goto	LCD
LCD_INCPLACE
	INCF	LCD_PLACE,F
LCD
	call	send_lcd_command
	call	send_lcd_data
	return

LCD_RWT
	BCF	LCD_RW
	movlw	b'11110000'
	iorwf	PORTC,f	;установим все биты в "1", операция ИЛИ
	MOVF	LCD_LCD,W
	andwf	PORTC,f
	BSF	LCD_E
	CALL	tm50		;пауза 50 мкс
	BCF	LCD_E
	movlw	b'11110000'
	iorwf	PORTC,f	;установим все биты в "1", операция ИЛИ
	return

LCD_INIT
;<POWER ON>
;according to datasheet, we need at least 40ms after power rises above 2.7V
;RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
; 0  0   0   0   1   1  n/c n/c n/c n/c
	call	tm40000	;пауза 40 мс
	BCF	LCD_RS_OUT; Регистр комманд
	BCF	LCD_RW
	BCF	LCD_E
	MOVLW	b'00111111'	; 8-ми разрядная шина
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	call	tm40000	;пауза 40 мс

;RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
; 0  0   0   0   1   1  n/c n/c n/c n/c
	MOVLW	b'00111111'	; 8-ми разрядная шина
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	call 	tm100	;пауза 100 мкс

;RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
; 0  0   0   0   1   1  n/c n/c n/c n/c
	MOVLW	b'00111111'	; 8-ми разрядная шина
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200	;пауза 200 мкс

;RS R/W DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
; 0  0   0   0   1   0  n/c n/c n/c n/c
	CALL	tm200	;пауза 200 мкс
	MOVLW	b'00101111'	; 4-х разрядная шина
	MOVWF	LCD_LCD
	CALL	LCD_RWT

;RS R/W DB7 DB6 DB5 DB4 ;Function Set Command (0x20)
; 0  0   0   0   1   0
; 0  0   N   F   X   X  ;where N = Number of `lines'
                        ; 0 for 1/8 duty cycle -- 1 `line'
                        ; 1 for 1/16 duty cycle -- 2 `lines'
                        ;F = font
                        ; 0 for 5x8 dot matrix
                        ; 1 for 5x11 dot matrix
                        ;X = don't care
	MOVLW	b'00101111'	; 4-х разрядная шина
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'10001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200	;пауза 200 мкс

;Display Off, Cursor Off, Blink Off
;command(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF)
	MOVLW	b'00001111'	; выключить дисплей
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'10001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200	;пауза 200 мкс

;command(LCD_ENTRYMODESET | LCD_MOVERIGHT)
	MOVLW	b'00001111'	; флаги
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'01101111'	;
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200	;пауза 200 мкс

;command(LCD_DISPLAYCONTROL | LCD_DISPLAYON)
	MOVLW	b'00001111'	; включить дисплей
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11001111'	; включить дисплей
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200	;пауза 200 мкс

#ifdef USE_CUSTOM_CHARS
LCD_INIT_CUSTOM_CHARS
;load custom characters

	clrf	R0
_next_custom_char
;command(LCD_SETCGRAMADDR | (location << 3))
	movf	R0,w
	movwf	LCD_PLACE
	bcf		STATUS,C
	rlf		LCD_PLACE
	rlf		LCD_PLACE
	rlf		LCD_PLACE,w
	movwf	_tmp
	addlw	0x40
	movwf	LCD_PLACE

;send command (LCD_PLACE)
	call	send_lcd_command

	clrf	R1
_next_custom_char_byte
	pageselw get_custom_char_data
	call	get_custom_char_data
	movwf	LCD_LCD
	pageselw $
;send data (LCD_LCD)
	call	send_lcd_data
	incf	_tmp,f
	incf	R1,f
	movf	R1,w
	andlw	0x08
	btfsc	STATUS,Z
	goto	_next_custom_char_byte

	incf	R0,f
	movf	R0,w
	andlw	0x08
	btfsc	STATUS,Z
	goto	_next_custom_char
#endif
	RETURN

LCD_CLEAR
	call	tm4100	; даем немного отдохнуть
	BCF	LCD_RS_OUT; Регистр комманд
	BCF	LCD_RW
	MOVLW	b'00001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'00011111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	call	tm4100
	RETURN

;***********************************************************************************
ALL_TEMP ; подпрограмма опроса температурных датчиков
	movlw	.3
	movwf	_tmp
	bcf	fTIMER_FL
	bsf	PCLATH,3	; страница памяти 1
	movlw	h'30'		; адрес младшего байта в EEPROM 1 датчик
	call	Set_ROM
	call	READ_TEMP
	bcf	PCLATH,3	; страница памяти 0
	btfsc	CRC_OK		; если флаг установлен - данные верны
	goto	NEXT1
	decf	_tmp,f
	movlw	0x88		; magic word for printing '----'
	movwf	fTEMPER_L
	movwf	fTEMPER_H
NEXT1	bcf	STATUS,IRP
	movlw	TEMPER1_L
	movwf	FSR
	call	SAVE_T
	call	tm4100
	bsf	PCLATH,3	; страница памяти 1
	movlw	h'38'		; адрес младшего байта в EEPROM 2 датчик
	call	Set_ROM
	call	READ_TEMP
	bcf	PCLATH,3	; страница памяти 0
	btfsc	CRC_OK
	goto	NEXT2
	decf	_tmp,f
	movlw	0x88		; magic word for printing '----'
	movwf	fTEMPER_L
	movwf	fTEMPER_H
NEXT2	bcf	STATUS,IRP
	movlw	TEMPER2_L
	movwf	FSR
	call	SAVE_T
	call	tm4100
	bsf	PCLATH,3	; страница памяти 1
	movlw	h'40'		; адрес младшего байта в EEPROM 3 датчик
	call	Set_ROM
	call	READ_TEMP
	bcf	PCLATH,3	; страница памяти 0
	btfsc	CRC_OK
	goto	NEXT3
	decf	_tmp,f
	movlw	0x88		; magic word for printing '----'
	movwf	fTEMPER_L
	movwf	fTEMPER_H
NEXT3	bcf	STATUS,IRP
	movlw	TEMPER3_L
	movwf	FSR
	call	SAVE_T
	
	bcf	skipTemp	; to do configuration bit for it
	;movf	_tmp,w
	;btfsc	STATUS,Z
	;bsf	skipTemp

	btfss	ZUMER_ON,1
	return
				; проверяем температуру двигателя на предмет перегрева
	BANK1
	btfsc	TEMPER3_H,7	; если значение отрицательно - выходим
	goto	Z10
	movlw	0x04		; 102 градуса
	subwf	TEMPER3_H,w
	btfss	STATUS,C
	goto	Z10
	BANK0
	movlw	.01
	goto	LCD_ERROR
	
Z10	BANK0
	return


SAVE_T
	movf	fTEMPER_L,w
	movwf	INDF
	incf	FSR,f
	movf	fTEMPER_H,w
	movwf	INDF
	return

READ_T
	movf	INDF,w
	movwf	R3
	incf	FSR,f
	movf	INDF,w
	movwf	R4
	return

;***********************************************************************************
; Т А Й М Е Р Ы
;
tm40000	clrf	LCD_CT1	; таймер 40 мс
	clrf	LCD_CT2
	goto	timer1

tm4100	MOVLW	H'00'	; таймер 4.1 мс
	MOVWF	LCD_CT1
	MOVLW	H'e5'
	MOVWF	LCD_CT2
	goto	timer1

tm250	MOVLW	H'55'	; таймер 250 мкс
	MOVWF	LCD_CT1
	MOVLW	H'FE'
	MOVWF	LCD_CT2
	goto	timer1

tm200	MOVLW	H'B5'	; таймер 200 мкс
	MOVWF	LCD_CT1
	MOVLW	H'FE'
	MOVWF	LCD_CT2
	goto	timer1

tm100	MOVLW	H'54'	; таймер 100 мкс
	MOVWF	LCD_CT1
	MOVLW	H'ff'
	MOVWF	LCD_CT2
	goto	timer1

tm50	MOVLW	H'A0'	; таймер 50 мкс
	MOVWF	LCD_CT1
	MOVLW	H'FF'
	MOVWF	LCD_CT2
	goto	timer1

tm05	MOVLW	H'FB'	; таймер 5 мкс
	MOVWF	LCD_CT1
	MOVLW	H'FF'
	MOVWF	LCD_CT2
	nop

timer1	incfsz	LCD_CT1,F
	goto	timer1

	incfsz	LCD_CT2,F
	goto	timer1

	return

;====================================================================================
EEREAD		;Типовая процедура чтения байта из EEPROM
	BANK3
	bcf	EECON1,EEPGD
	bsf	EECON1,RD	;Читаем
	BANK2
	movf	EEDATA,W	;Из EEDATA в W
	incf	EEADR,f
	BANK0
	return

LCD_CLEAR_HALF_UP:
	movlw	0x80
	goto	_lcd_clear_half1
LCD_CLEAR_HALF_DOWN:
	movlw	0xC0
_lcd_clear_half1
	movwf	LCD_PLACE
	movlw	.15
	movwf	_tmp
    	movlw	_SPACE
	movwf	LCD_LCD
	call 	LCD
_lcd_clear_half2
	call 	LCD_INCPLACE
	decfsz 	_tmp, F
	goto 	_lcd_clear_half2
	return

	
;	org 0x06F0
;=====================================================================================
LCD_ERROR:
	; отображение критического параметра
	; используем локально рег.
	; fTRY - параметр отображение, передается в процедуру через W
	; _tmp,0 - температура
	; _tmp,1 - сервис счетчик 1
	; _tmp,2 - сервис счетчик 2
	; _tmp,3 - сервис счетчик 3
	; _tmp,4 - сервис счетчик 4
	; _tmp,5 - скорость
	; _tmp,6 - тахометр
	; _tmp,7 - моточасы
	; fTEMP - время отображение экрана
;=====================================================================================
	movwf	fTRY
	clrf	fTEMP
	
	bsf	FL_zumer
	clrf	PCLATH
	call	LCD_CLEAR
	movlw	0x82
	movwf	LCD_PLACE
	MSLOVO_XXX  0xF0, 0x09	; внимание
	
	btfsc	fTRY,0		; температура шкалит
	goto	_Z30
	gotop	Z30
_Z30
	bsf	FL_zumerВ2
	movlw	0xBF
	movwf	LCD_PLACE
	MSLOVO_YYY  0x6B, 0x0B
	movlw	TEMPER3_L	; покажем текущую температуру
	movwf 	FSR
	call	READ_T
	movlw 	0xCC
	bsf	PCLATH,3	; страница памяти 1
	call	LCD_TEMP2
	call	LCD_PAUSE

Z30
	bsf	FL_zumerВ3
	btfss 	fTRY,1		; сервис счетчик 1
	goto	Z31
	call	LCD_CLEAR_HALF_DOWN
	movlw	0xBF + .2
	movwf	LCD_PLACE
	MSLOVO_XXX  0x9F, 0x0E	; ТО масло двиг
	call	LCD_PAUSE

Z31	btfss	fTRY,2		; сервис счетчик 2
	goto	Z32
	call	LCD_CLEAR_HALF_DOWN
	movlw	0xBF + .2
	movwf	LCD_PLACE
	MSLOVO_XXX  0xAF, 0x0D	; ТО масло АКПП
	call	LCD_PAUSE

Z32	btfss	fTRY,3		; сервис счетчик 3
	goto	Z33
	call	LCD_CLEAR_HALF_DOWN
	movlw	0xBF + .4
	movwf	LCD_PLACE
	MSLOVO_XXX  0xCF, 0x08	; ТО свечи
	call	LCD_PAUSE

Z33	btfss	fTRY,4		; сервис счетчик 4
	goto	Z34
	call	LCD_CLEAR_HALF_DOWN
	movlw	0xBF + .3
	movwf	LCD_PLACE
	MSLOVO_XXX  0xBF, 0x0A	; ТО возд.фильтр
	call	LCD_PAUSE

Z34
Z35
Z36
	btfss	fTRY,7		; счетчик моточасов
	goto	Z37
	call	LCD_CLEAR_HALF_DOWN
	movlw	0xBF + .2
	movwf	LCD_PLACE
	MSLOVO_YYY  0xC7, 0x0E	; моточасы
	call	LCD_PAUSE

;	goto	main1
Z37
	;movlw	h'01'		; инициализация таймера 2 сек.
	;movwf	fTIMER
	call	LCD_CLEAR
	bsf	SEC2_OK
	return

LCD_PAUSE:
	btfss	SEC2_OK
	goto	$-1
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto 	LCD_PAUSE
	clrf	fTEMP
	return
	    
LCD_AVERAGE_SPEED
	movwf	LCD_PLACE

	bsf	PCLATH,3		;страница памяти 1
	call	get_odo00		;R0:R1:R2 - одометр * 10 км (с точностью 0.1 км)
	bcf	PCLATH,3		;страница памяти 0
	
#ifdef AVERAGE_MIN_DIST
	movlw	low AVERAGE_MIN_DIST
	subwf   R2,w

	movlw   high AVERAGE_MIN_DIST
	btfss   STATUS,C
	addlw   .1
	subwf   R3,w

	btfss   STATUS,C
	goto	lcd_av_speed_1 ; '----', если расстояние меньше минимального
#endif
	
	;R0:R1:R2 - одометр * 10 км (с точностью 0.1 км)
	;move to R3:R4:R5
	movf	R0,w
	movwf	R3
	movf	R1,w
	movwf	R4
	movf	R2,w
	movwf	R5
	;3600/2 - R13:R14:R15
	clrf	R13
	movlw	high .1800
	movwf	R14
	movlw	low .1800
	movwf	R15

	bsf	PCLATH,3	; страница памяти 1
	call	mul24_24	; R0:R1:R2:R3:R4:R5 = R3:R4:R5 * R13:R14:R15
	
	FSR_ABC	time_H,time_H_B,time_H_C
	movf	INDF,w
	movwf	R13
	incf	FSR,F
	movf	INDF,w
	movwf	R14
	incf	FSR,F
	movf	INDF,w
	movwf	R15
	
	;R0:R1:R2:R3:R4:R5 = R0:R1:R2:R3:R4:R5 / R13:R14:R15
	call	div_48by24
	iorlw	0x00
	btfsc	STATUS,Z
	goto	lcd_av_speed_1

	; reorder R3:R4:R5 -> R5:R4:R3 (swap R3 and R5)
	MOVF  R3,W ; W:=A
	XORWF R5,W ; W:=A^B
	XORWF R3,F ; X:=((A^B)^A)=B
	XORWF R5,F ; Y:=((A^B)^B)=A

	;R5:R4:R3 - average speed
	bcf	PCLATH,3	; страница памяти 0
	call	B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2
	prv_wi	R1
	xorlw	0x30
	btfsc	STATUS,Z
	goto	lcd_av_speed_4

	prv_w	R0
	bsf	ZERO
	call 	ZERO_

	prv_wi	R1
	call 	ZERO_

	prv_w	R1
	call 	ZERO_

	prv_wi	R2
	call 	LCD
	goto	lcd_av_speed_2

lcd_av_speed_4
	xorlw	0x30
	decf	LCD_PLACE
	bsf	ZERO
	call 	ZERO_

	prv_w	R1
	call 	ZERO_

	prv_wi	R2
	call 	LCD

	movlw	_POINT		;запятая
	movwf	LCD_LCD
	call 	LCD_INCPLACE

	prv_w	R2
	call 	LCD_INCPLACE

lcd_av_speed_2
#ifdef USE_CUSTOM_CHARS
	movlw	_kmh0
	call	LCD_
	movlw	_kmh1
	call	LCD_
	movlw	_SPACE
	call	LCD_
	call	LCD_INCPLACE
#else
	movlw	.31
	movwf	PCLATH		; страница памяти 3
	call	SLOVO19		; км/ч
#endif

	return
lcd_av_speed_1
	movlw	0x2D
	movwf	LCD_LCD
	call 	LCD		; -
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	goto	lcd_av_speed_2


;=====================================================================================
;=====================================================================================
;=================          С Т Р А Н И Ц А  1           =============================
;=====================================================================================
;=====================================================================================
;=====================================================================================

     	org	0x800	; страница памяти 1

;==============================================================================================
;Подпрограммы для работы с шиной I2C. 100 kHz ДЛЯ 20 мГц
;==============================================================================================
; Требуется 2 байта ОЗУ - i2c_byte - в него помещается байт,
; который необходимо вывести на шину, в нем оказывается байт,
; считанный с шины; tmp - счетчик, используемый в цикле. */
;==============================================================================================
;char i2c_byte; /* рабочий регистр */
;char tmp;      /* счетчик */

TIME_R				; подпрограмма считывания таймера
	bsf	time_OK
	CALL	DSTIME_READ_BYTES
	btfsc	EE_DATA,7	; проверка лояльности часов
	call	INIT_TIMER
	return

TIME_W				; подпрограмма записи таймера
	clrf	EE_DATA		; очистка секунд
	call	DSTIME_SAVE_BYTES
	return

INIT_TIMER
	movlw	b'10010000'
	movwf	EE_DATA
	clrf	MINUTES
	movlw	0x12
	movwf	HOURS
	movlw	0x06
	movwf	DAYOFWEEK
	movlw	0x21
	movwf	DATE
	movlw	0x05
	movwf	MONTH
	movlw	0x21
	movwf	YEAR
	call	DSTIME_SAVE_BYTES
	return

;#define NEW_I2C	    ; use MAXIM I2C routines https://www.maximintegrated.com/en/design/technical-documents/app-notes/3/3921.html

#ifdef NEW_I2C

SDA_OUTPUT macro
	BANK1
	bcf	TRIS_SDA		; SDA low (output)
	BANK0
	endm
SDA_INPUT macro
	BANK1
	bsf	TRIS_SDA		; SDA high (input)
	BANK0
	endm
SCL_OUTPUT macro
	BANK1
	bcf	TRIS_SCL		; SCL low (output)
	BANK0
	endm
SCL_INPUT macro
	BANK1
	bsf	TRIS_SCL		; SCL high (input)
	BANK0
	endm
	
_I2C_DELAY	macro
	goto	$+1
	endm
	
_I2C_START MACRO
	bsf	SDA			; SDA high
	nop
	SCL_INPUT
	bcf	SDA			; SDA low (start)
	ENDM

_I2C_STOP MACRO				; assumes SCL is high on entry
	bcf	SDA			; SDA low
	nop
	nop
	SCL_INPUT
	bsf	SDA			; SDA high
	ENDM

DSTIME_READ_BYTES
	_I2C_START
	movlw	0D0h			; slave address + write
	call	write_RTC
	movlw	0			; set word address to seconds register
	call	write_RTC
	_I2C_START
	movlw	0D1h			; slave address + read
	call	write_RTC
	call	read_RTC		; read the seconds data
	movwf	EE_DATA			; save it
	call	ack
	call	read_RTC		; and so on
	movwf	MINUTES
	call	ack
	call	read_RTC
	movwf	HOURS
	call	ack
	call	read_RTC
	movwf	DAYOFWEEK
	call	ack
	call	read_RTC
	movwf	DATE
	call	ack
	call	read_RTC
	movwf	MONTH
	call	ack
	call	read_RTC
	movwf	YEAR
	call	nack
	_I2C_STOP
	return

DSTIME_SAVE_BYTES
	_I2C_START
	movlw	0D0h			; slave address + write
	call	write_RTC
	movlw	0			; set word address to seconds register
	call	write_RTC
	movf	EE_DATA, W
	call	write_RTC
	movf	MINUTES, W
	call	write_RTC
	movf	HOURS, W
	call	write_RTC
	movf	DAYOFWEEK, W
	call	write_RTC
	movf	DATE, W
	call	write_RTC
	movf	MONTH, W
	call	write_RTC
	movf	YEAR, W
	call	write_RTC
	_I2C_STOP
	return

;---- Read RTC into W  ----
read_RTC:
	SDA_INPUT

	movlw	08h			; send 8 bits
	movwf	tTIME

	bcf	SCL			; clock data out
	SCL_OUTPUT

	clrf	_i2c_byte		; clear var
	rlf	_i2c_byte, 1		; rotate carry in
	clrf	_i2c_byte		; clear var again

I2C_read_loop:
	rlf	_i2c_byte, 1

	SCL_INPUT

	btfsc	SDA
	bsf	_i2c_byte, 0		; if data out = 1, set bit

	bcf	SCL

	SCL_OUTPUT
	
	decfsz	tTIME, 1
	goto	I2C_read_loop

	movf	_i2c_byte, W

	return

;---- ACK read (assumes SCL=0 on entry) ----
ack:
	bcf	SDA

	SDA_OUTPUT

	SCL_INPUT
	nop
	bcf	SCL

	SCL_OUTPUT
	
	return

;---- NACK read (assumes SCL = 0 on entry) ----
nack:
	bsf	SDA

	SDA_OUTPUT

	SCL_INPUT

	bcf	SCL
	
	SCL_OUTPUT

	return

;--- Write the byte in W to RTC ---
;---- assumes CE is asserted ----
write_RTC:
	movwf	_i2c_byte			;Save the data
;
;--- Do a I2C bus write of byte in 'TMP' ---
;
I2C_write:

	SDA_OUTPUT

	movlw	08h			; send 8 bits
	movwf	tTIME

	bcf	SCL
	SCL_OUTPUT

I2C_w_loop:
	bcf	SDA			; assume data out is low
	btfsc	_i2c_byte, 7
	bsf	SDA			; if data out = 1, set bit
	nop

	SCL_INPUT
	rlf	_i2c_byte, 1
	bcf	SCL			; clock it in
	SCL_OUTPUT
	decfsz	tTIME, 1
	goto	I2C_w_loop

	SDA_INPUT

	bcf	SCL
	SCL_OUTPUT
	nop
	SCL_INPUT
	
	SDA_OUTPUT
	; if(sda) printf("Ack bit missing  %02X\n",(unsigned int)d);
	bcf	SCL
	SCL_OUTPUT

	return
#else

DSTIME_READ_BYTES
	call  	i2c_start
	movlw	0xD0
   	movwf	_i2c_byte 	; control байт с битом WR
	call	i2c_write
	btfss	time_OK
	return
	clrf	_i2c_byte   	; байт адреса памяти
	call 	i2c_write	;
 	btfss	time_OK
	return
	call  	i2c_restart;
	movlw	0xD1
	movwf	_i2c_byte    	; control байт с битом RD
	call  	i2c_write
	btfss	time_OK
	return

	; чтение байт с i2c в RAM
	movlw	EE_DATA		; начальный адрес в RAM
	movwf	FSR
	movlw	.7		; количество байт
	movwf	tTIME
	goto	$+2
_i2c_read
	call	i2c_send_ack
	call  	i2c_read
	movf 	_i2c_byte,W
	movwf	INDF
	incf	FSR,f

	decf	tTIME,f
	btfss	STATUS,Z
	goto	_i2c_read

	call  	i2c_stop
	return

DSTIME_SAVE_BYTES
	call  	i2c_start
	movlw	0xD0
	movwf	_i2c_byte	; control байт с битом WR
	call	i2c_write

   	clrf	_i2c_byte	; младший байт адреса памяти
	call	i2c_write

	movlw	EE_DATA		; начальный адрес в RAM
	movwf	FSR
	movlw	.7		; количество байт
	movwf	tTIME

_i2c_write
	movf	INDF,W
	movwf	_i2c_byte	; записываемые данные
	call	i2c_write
	incf	FSR,f

	decf	tTIME,f
	btfss	STATUS,Z
	goto	_i2c_write

	call	i2c_stop

	return

;==============================================================================================
i2c_delay	macro
	goto	$+1
	endm

;==============================================================================================
i2c_restart
 	bsf 	SDA     	; SDA - 1
	i2c_delay
	bsf 	SCL     	; SCL - 1
	i2c_delay
	bcf 	SDA     	; SDA - 0
	i2c_delay
  	bcf 	SCL    		; SCL - 0
	return

i2c_start
	bsf 	SCL     	; SCL - 1
	i2c_delay
 	bsf 	SDA     	; SDA - 1
	i2c_delay
	bcf 	SDA     	; SDA - 0
	i2c_delay
  	bcf 	SCL    		; SCL - 0
	return

;==============================================================================================
i2c_stop
	bcf	SDA    		; SDA - 0
	i2c_delay
  	bsf 	SCL     	; SCL - 1
	i2c_delay
  	bsf 	SDA     	; SDA - 1
	return

;==============================================================================================
i2c_write
	movlw	0x08
	movwf	_tmp
  	bcf 	SCL            	; SCL - 0
	i2c_delay
send_bit
  	bsf 	SDA		; SDA - 1 выводим 1 на SDA
	i2c_delay
	btfss	_i2c_byte, 7	; бит данных - 1 ?
 	bcf 	SDA          	; нет - выводим 0 на SDA
	i2c_delay
  	bsf 	SCL     	; SCL - 1
	i2c_delay
	rlf 	_i2c_byte, F    ; сдвигаем влево байт данных
	bcf 	SCL          	; SCL - 0
	i2c_delay
	decfsz 	_tmp, F     	; конец цикла ?
	goto 	send_bit        ; нет - переход


	bsf 	SDA          	; осв. SDA SDA - 1
	BANK1
 	bsf 	TRIS_SDA     	; SDA - на ввод
	BANK0
	clrf	_tmp		; используем регистр как таймер ожидания ACK (256 циклов)
WaitForACK
	incf	_tmp, f
	btfsc	STATUS, Z
	goto	No_ACK_Rec
	btfsc	SDA		; проверяем наличие импульса подтверждения
	goto	WaitForACK	; продолжаем ждать
	goto	$+3

No_ACK_Rec			; не дожждались
	bcf	time_OK		; сбросим флаг лояльности показаний
	return

	bsf 	SCL     	; SCL - 1
	BANK1
   	bcf 	TRIS_SDA       	; SDA - вывод
	BANK0
	bcf 	SCL    		; SCL - 0

	return


;==============================================================================================

i2c_read
	movlw 0x08
	movwf _tmp
	BANK1
  	bsf 	TRIS_SDA     	; SDA - на ввод
	BANK0
   	bcf 	SCL          	; SCL - на 0
	i2c_delay

read_bit
	rlf 	_i2c_byte, F    ; сдвигаем байт влево
	bsf 	SCL     	; SCL - 1
	i2c_delay
	bsf 	_i2c_byte, 0    ; устанавливаем бит в 1 ; -NOP
	NOP
	btfss 	SDA	       	; SDA - 1?
	bcf 	_i2c_byte, 0   	; нет - устанавливаем бит в 0
	bcf 	SCL   		; SCL - 0
	i2c_delay
	decfsz 	_tmp, F     	; конец цикла ?
	goto 	read_bit        ; нет - переход

	BANK1
	bcf 	TRIS_SDA       	; SDA - вывод
	BANK0

	return

i2c_send_ack
	bcf 	SCL   		; SCL - 0
	i2c_delay
	bcf	SDA		; SDA - 0
	i2c_delay
	bsf 	SCL     	; SCL - 1
	i2c_delay
	bcf 	SCL   		; SCL - 0
	return
#endif
;-----------------------------------------------------------------------------
;-----------------------------------------------------------------------------
;-------------    ПОДПРОГРАММЫ РАБОТЫ С ШИНОЙ 1WARE   ------------------------
;-----------------------------------------------------------------------------
;-----------------------------------------------------------------------------


READ_TEMP:
;	call  Get_Temperature 	; Забираем температуру.


;==============================================================================================
; Возвращает:
; Если OK, то в регистрах fTEMPER_L и fTEMPER_H температура  и CRC_OK = 1.
; Если ошибк  CRC, то CRC_OK = 0.
Get_Temperature:
        movlw   d'3' ; Делаем 3 попытки чтения.
        movwf   fTRY
GT_Try: ; Цикл попыток чтения.
; Сбрасываем все приборы.
        call    Reset_1Wire
	call	Match_ROM
; Выводим команду транспортного уровня "Чтение блокнотной памяти".
        movlw   0xBE
        call    RW_Byte
; Считываем данные и вычисляем CRC.
        clrf    fCRC
        movlw   0xFF
        call    RW_Byte ; byte 0.
        movwf   fTEMPER_L ; Младший байт температуры.
        call    DO_CRC
        movlw   0xFF
        call    RW_Byte ; byte 1.
        movwf   fTEMPER_H ; Старший байт температуры.
        call    DO_CRC
; Далее ненужные байты (читаем только из-з  CRC).
        movlw   0xFF
        call    RW_Byte ; byte 2.
        call    DO_CRC
        movlw   0xFF
        call    RW_Byte ; byte 3.
        call    DO_CRC
        movlw   0xFF
        call    RW_Byte ; byte 4.
        call    DO_CRC
        movlw   0xFF
        call    RW_Byte ; byte 5.
        call    DO_CRC
; Далее данные, необходимые для более точного вычисления температуры.
        movlw   0xFF
        call    RW_Byte ; byte 6.
        movwf   fCNT_REM
        call    DO_CRC
        movlw   0xFF
        call    RW_Byte ; byte 7.
        movwf   fCNT_PER_C
        call    DO_CRC
; Читаем CRC.
        movlw   0xFF
        call    RW_Byte ; byte 8 (CRC).
        subwf   fCRC,1
        btfsc   STATUS,Z
        goto    GT_CRC_OK
; Если неправильная CRC, делаем следующую попытку.
        movlw   0
        call    delay
        decfsz  fTRY,f
        goto    GT_Try
; Если попытки исчерпаны.
      bcf     CRC_OK
      goto	NEXT_TEMP
; Если CRC - ok.
GT_CRC_OK:
      bsf     CRC_OK
	btfss	ZUMER_ON,7
	goto	GT_B
;------------ Вычисление температуры ---------------
; R3:R2 = COUNT_PER_C - COUNT_REMAIN;
        movf    fCNT_REM,w
        subwf   fCNT_PER_C,w   ; f-w
        movwf   R2
        clrf    R3
; R1:R0 = R3:R2 * 10;
        movlw   0x0A
        movwf   R4
        clrf    R5
        call    mul_16 ; R1:R0 = R3:R2 * R5:R4

; R1:R0 = R1:R0 / COUNT_PER_C;
        movf    R0,w
        movwf   R2
        movf    R1,w
        movwf   R3

        movf    fCNT_PER_C,w
        movwf   R4
        clrf    R5
        call    div_16 ; R1:R0 = R3:R2 / R5:R4

; R7:R6 = R1:R0 - 2;
        movlw   0xFE
        addwf   R0,f
        btfss   STATUS,C
        decf    R1,f   ; Перенос.
; Сохраняем.
        movf    R0,w
        movwf   R6
        movf    R1,w
        movwf   R7

; R1:R0 = TEMP_READ * 10;
        rrf     fTEMPER_H,0
        rrf     fTEMPER_L,0 ; В w получился TEMP_READ.
        movwf   R2
        movf    fTEMPER_H,w
        movwf   R3
        movlw   d'10'
        movwf   R4
        clrf    R5
        call    mul_16 ; R1:R0 = R3:R2 * R5:R4

; fTEMPER = R1:R0 + R7:R6;
        movf    R6,w
        addwf   R0,f
        btfsc   STATUS,C
        incf    R1,f   ; Перенос.
        movf    R7,w
        addwf   R1,f
; Сохраняем.
	movf    R0,w
	movwf   fTEMPER_L
	movf    R1,w
	movwf   fTEMPER_H
	goto	NEXT_TEMP

GT_B
	movf     fTEMPER_H,w
     	movwf     R3
     	movf     fTEMPER_L,w
     	movwf     R2
     	btfss     fTEMPER_H,7
     	goto     $+3
     	comf     R2,f
     	comf     R3,f
     	movlw   0x0A
     	movwf   R4
     	clrf    R5
     	call    mul_16 ; R1:R0 = R3:R2 * R5:R4

     	movf     R1,w
     	movwf     R3
     	movf     R0,w
     	movwf     R2
     	movlw   0x10
     	movwf   R4
     	clrf    R5
     	call    div_16 ; R1:R0 = R3:R2 / R5:R4

     	btfss     fTEMPER_H,7
     	goto     $+3
     	comf     R1,f
     	comf     R0,f

     	movf     R1,w
     	movwf     fTEMPER_H
     	movf     R0,w
     	movwf     fTEMPER_L


NEXT_TEMP
	call	Reset_1Wire
	call	Match_ROM
        movlw 0x44			; новый запрос преобразования температуры
        call  RW_Byte
	return

;-----------------------------------------------------------------------------
Match_ROM:
; Выдаем команду "Совпадение ПЗУ".
	movlw   0x55
	call    RW_Byte

	bsf	STATUS,IRP
	movlw	low fROM_ID0
	movwf	FSR

	clrf	R0
_match_rom1
	movf    INDF,w
	call    RW_Byte	;bytes 0-7
	incf    FSR,f
	incf    R0,f
	movf    R0,w
	xorlw   0x08
	btfss   STATUS,Z
	goto    _match_rom1
	bcf	STATUS,IRP
	return

;-----------------------------------------------------------------------------
Set_ROM:
; Загрузка сетевого номер активного датчик для работы на  линии.

	bsf	STATUS,IRP
	BANK2
	movwf	EEADR			;Выбираем адрес в области EEPROM
	movlw	low fROM_ID0
	movwf	FSR			;Выбираем адрес регистра в оперативной памяти
	movlw	low fROM_ID7
	pagesel READ_BYTES
	call	READ_BYTES
	pagesel $
	BANK0
	bcf	STATUS,IRP
	return

;-----------------------------------------------------------------------------
; Возвращает:
; Если OK, то содержимое ПЗУ в регистрах fROM_ID0:fROM_ID7.
Get_ROM_ID:
	bcf     CRC_OK
	clrf    fCRC
; Сбрасываем все приборы на линии.
	call    Reset_1Wire
; Выдаем команду "Чтение ПЗУ".
	movlw   0x33
	call    RW_Byte
; Читаем ПЗУ.
	bsf	STATUS,IRP
	movlw	low fROM_ID0
	movwf	FSR

	clrf	R0
_get_rom_id1
	movlw   0xFF
	call    RW_Byte ; byte 0-6
	movwf	INDF
	call    DO_CRC
	incf	FSR,f
	incf	R0,f
	movf	R0,w
	xorlw	0x07
	btfss	STATUS,Z
	goto	_get_rom_id1

; Читаем CRC.
	movlw   0xFF
	call    RW_Byte ; byte 7.
	movwf	INDF

	subwf   fCRC,1
	btfsc   STATUS,Z
 	bsf     CRC_OK

	bcf	STATUS,IRP
	return

;-----------------------------------------------------------------------------
W_LOW
	bcf   T_1WIRE
	bsf	STATUS,RP0
	bcf	TRISA,5		; порт на выход
	bcf	STATUS,RP0
        return
;-----------------------------------------------------------------------------
W_HIGH
	bsf	STATUS,RP0
	bsf	TRISA,5		; порт на вход
	bcf	STATUS,RP0
        return
;-----------------------------------------------------------------------------
Reset_1Wire:   ; Сбрасываем линию на около 500 мкс.
	call	W_LOW
        movlw   0
        call    delay ; 154 мкс.
        call    delay ; 154 мкс.
        call    delay ; 154 мкс.
        call    delay ; 154 мкс.

	call    W_HIGH
; Пропускаем импульс присутствия.
        movlw   0
        call    delay ; 154 мкс.
        call    delay ; 154 мкс.
        call    delay ; 154 мкс.
        call    delay ; 154 мкс.
        return
;-----------------------------------------------------------------------------
; Ф-ция ввод /вывод  н  линию 1-Wire.
; Выводим из W и читаем в W.
RW_Byte:
        movwf   fTEMP
        movlw   d'8' ; 8 бит.
        movwf   fBIT_CNT
RBLoop:
	call	W_LOW ; Обнуляем выход.
; Ждем 2 мкс.
	goto 	$+1
	goto 	$+1
	goto 	$+1
	goto 	$+1
	goto 	$+1
        btfsc   fTEMP,0
	call    W_HIGH ; Устанавливаем выход.
        rrf     fTEMP,1
; Ждем ~12 мкс.
        movlw   18
        call    delay
        bcf     fTEMP,7 ; Принимаем в тот же TEMP, из которого передаем.
        btfsc   T_1WIRE
        bsf     fTEMP,7
; Далее даем время на  освобождение линии ведомым.
        movlw   d'100'
        call    delay
; Отпускаем линию.
	call    W_HIGH
; Чуть-чуть даем отдохнуть (для медленных эмуляторов). Для Dallas не нужно.
        movlw   d'1'      ; Впоследствии можно удалить при нормальной работе.
        call    delay
; Если 8 бит, то выходим.
        decfsz  fBIT_CNT,1
        goto    RBLoop
; Принятый байт в W.
        movf    fTEMP,0
        return

;-----------------------------------------------------------------------------
DO_CRC:
; Процедура  обновления CRC. Параметр в W.
        movwf   fSAVE_W
        movlw   0x08
        movwf   fBIT_CNT
        movf    fSAVE_W,w
DoCRC_Loop:
        xorwf   fCRC,w
        movwf   fTEMP
        rrf     fTEMP,w
        movf    fCRC,w
        btfsc   STATUS,0
        xorlw   0x18
        movwf   fTEMP
        rrf     fTEMP,w
        movwf   fCRC
        bcf     STATUS,0
        rrf     fSAVE_W,f
        movf    fSAVE_W,w
        clrwdt
        decfsz  fBIT_CNT,f
        goto    DoCRC_Loop
        return
;-----------------------------------------------------------------------------
; Большая задержка.
Big_delay:
        movwf   fCOUNTER2
        clrf    fCOUNTER
BD_Loop94:
        decfsz  fCOUNTER,f
        goto    BD_Loop94
        decfsz  fCOUNTER2,f
        goto    BD_Loop94
        return
;-----------------------------------------------------------------------------
delay:
        movwf   fCOUNTER
D_Loop36:
        decfsz  fCOUNTER,1
        goto    D_Loop36
        return



;-----------------------------------------------------------------------------
;-----------------------------------------------------------------------------
;-----------------------------------------------------------------------------
	org 0x0A00
TIME_
	bcf	button_OK2
	movf	tTIME,w
	addwf	PCL,f
	nop		; коррекция
	goto	TIME_1 	; часов
	goto	TIME_2	; минут
	goto	TIME_3	; дней
	goto	TIME_4	; месяца
	goto	TIME_5	; года

	incf	DAYOFWEEK,f		; день недели
	movf	DAYOFWEEK,w
	xorlw	.8
	btfss	STATUS,Z
        return
 	movlw	.01
	movwf	DAYOFWEEK
        return
;-----------------------------------------------------------------------------

TIME_1	movf	HOURS,w
	andlw	b'00001111'  	;HOUR
	movwf	R0

	swapf	HOURS,w
	andlw	b'00001111'	; 10 HOUR
	movwf	R2

	incf	R0,f

	movf	R0,w
	xorlw	.10
	btfsc	STATUS,Z
	goto	INC_10HOUR

	movf	R2,w
	xorlw	.2
	btfss	STATUS,Z
	goto	INC_HOUR

	movf	R0,w
	xorlw	.4
	btfsc	STATUS,Z
	goto	INC_0HOUR
	goto	INC_HOUR

INC_10HOUR
	clrf	R0
	incf	R2,f

INC_HOUR
	swapf	R2,w
	iorwf	R0,w
	movwf	HOURS
        return

INC_0HOUR
	clrf	HOURS
        return

;-----------------------------------------------------------------------------
TIME_2	movf	MINUTES,w
	andlw	b'00001111'  	;HOUR
	movwf	R0

	swapf	MINUTES,w
	andlw	b'00001111'	; 10 HOUR
	movwf	R2

	incf	R0,f

	movf	R0,w
	xorlw	.10
	btfsc	STATUS,Z
	goto	INC_10MINUTES

INC_MINUTES
	swapf	R2,w
	iorwf	R0,w
	movwf	MINUTES
        return

INC_10MINUTES
	clrf	R0
	incf	R2,f
	movf	R2,w
	xorlw	.6
	btfsc	STATUS,Z
	goto	INC_0MINUTES
	goto	INC_MINUTES

INC_0MINUTES
	clrf	MINUTES
        return

;-----------------------------------------------------------------------------
TIME_3	movf	DATE,w
	andlw	b'00001111'  	;DATE
	movwf	R0

	swapf	DATE,w
	andlw	b'00001111'	; 10 DATE
	movwf	R2

	incf	R0,f

	movf	R0,w
	xorlw	.10
	btfsc	STATUS,Z
	goto	INC_10DATE

	movf	R2,w
	xorlw	.3
	btfss	STATUS,Z
	goto	INC_DATE

	movf	R0,w
	xorlw	.2
	btfsc	STATUS,Z
	goto	INC_0DATE
	goto	INC_DATE

INC_10DATE
	clrf	R0
	incf	R2,f

INC_DATE
	swapf	R2,w
	iorwf	R0,w
	movwf	DATE
        return

INC_0DATE
	movlw	.01
	movwf	DATE
        return
;-----------------------------------------------------------------------------
TIME_4	movf	MONTH,w
	andlw	b'00001111'  	;MONTH
	movwf	R0

	swapf	MONTH,w
	andlw	b'00001111'	; MONTH
	movwf	R2

	incf	R0,f

	movf	R0,w
	xorlw	.10
	btfsc	STATUS,Z
	goto	INC_10MONTH

	movf	R2,w
	xorlw	.1
	btfss	STATUS,Z
	goto	INC_MONTH

	movf	R0,w
	xorlw	.3
	btfsc	STATUS,Z
	goto	INC_0MONTH
	goto	INC_MONTH

INC_10MONTH
	clrf	R0
	incf	R2,f

INC_MONTH
	swapf	R2,w
	iorwf	R0,w
	movwf	MONTH
        return

INC_0MONTH
	movlw	.01
	movwf	MONTH
        return

;-----------------------------------------------------------------------------
TIME_5	movf	YEAR,w
	andlw	b'00001111'  	;YEAR
	movwf	R0

	swapf	YEAR,w
	andlw	b'00001111'	; YEAR
	movwf	R2

	incf	R0,f

	movf	R0,w
	xorlw	.10
	btfsc	STATUS,Z
	goto	INC_10YEAR

	movf	R2,w
	xorlw	.3
	btfss	STATUS,Z
	goto	INC_YEAR
	goto	INC_0YEAR

INC_10YEAR
	clrf	R0
	incf	R2,f

INC_YEAR
	swapf	R2,w
	iorwf	R0,w
	movwf	YEAR
        return

INC_0YEAR
	movlw	.24
	movwf	YEAR
        return

;-----------------------------------------------------------------------------
KURSOR_
	movf	tTIME,w
	addwf	PCL,f
	nop
; часы
	retlw	0x81
	retlw	0x84
	retlw	0x88
	retlw	0x8B
	retlw	0x8E
	retlw	0xC0
; пробег
	retlw	0xC5
	retlw	0xC6
	retlw	0xC7
	retlw	0xC8
	retlw	0xC9
	retlw	0xCA
; флаги
	retlw	0xC4
	retlw	0xC5
	retlw	0xC6
	retlw	0xC7
	retlw	0xC8
	retlw	0xC9
	retlw	0xCA
	retlw	0xCB

div24_16:
        CLRF R6
        CLRF R7
        MOVLW .24
        MOVWF R3
LOOPU2416
        RLF R2, F        ;shift dividend left to move next bit to remainder
        RLF R1, F        ;and shift in next bit of result
        RLF R0, F

        RLF R7, F        ;shift carry (next dividend bit) into remainder
        RLF R6, F

        RLF R3, F        ;save carry in counter, since remainder
                         ;can be 17 bit long in some cases (e.g.
                         ;0x800000/0xFFFF)

        MOVF R4, W       ;substract divisor from 16-bit remainder
        SUBWF R7, F
        MOVF R5, W
        BTFSS STATUS, C
        INCFSZ R5, W
        SUBWF R6, F

        SKPNC            ;if no borrow after 16 bit subtraction
        BSF R3, 0        ;then no no borrow in result. Overwrite
                         ;LOOPCOUNT.0 with 1 to indicate no
                         ;borrow.
                         ;if borrow did occur, LOOPCOUNT.0 will
                         ;hold the eventual borrow value (0-borrow,
                         ;1-no borrow)

        BTFSC R3, 0      ;if no borrow after 17-bit subtraction
        GOTO UOK46LL     ;skip remainder restoration.

        ADDWF R6, F      ;restore higher byte of remainder. (w
                         ;contains the value subtracted from it
                         ;previously)
        MOVF R4, W       ;restore lower byte of remainder
        ADDWF R7, F

UOK46LL
        CLRC             ;copy bit LOOPCOUNT.0 to carry
        RRF R3, F        ;and restore counter

        DECFSZ R3, f     ;decrement counter
        GOTO LOOPU2416   ;and repeat loop if not zero. carry
                         ;contains next quotient bit (if borrow,
                         ;it is 0, if not, it is 1).

        RLF R2, F        ;shift in last bit of quotient
        RLF R1, F
        RLF R0, F
        RETURN

;=====================================================================================
; подпрограмма умножения 2х байтных чисел R5:R4 на однобайтное R3
; params
;	R5:R4 	= number to multiply  (16bit)
;	R3	= multiplier (8bit)
;  	R0:R1:R2	= Product (24bit)
;
MUL16x8
	clrf	R0		;clear result
	clrf	R1		;
	clrf	R2		;
	movlw	.8
	movwf	R7	;bit counter set @ number of bits in multiplier

mul_L1				;main loop here
	rlf	R2,f		;rotate result left (*2)
	rlf	R1,f		;
	rlf	R0,f		;
	bcf	R2,0		;clear LSB of result after rotation
	btfss	R3,7		;test msb of multipier
	goto	dontAdd		;if bit is clear then dont add
	movfw   R4      	;Add the 16 bits of AA to product
	addwf   R2,f      	;
	skpnc			;
	incf	R1,f		;
	movfw   R5		;
	addwf   R1,f
	skpnc
	incf	R0,f
dontAdd
	rlf	R3,f		;rotate multiplier left
	decfsz	R7,f		;chk if finished all bits in multiplier
	goto	mul_L1

	return

;=====================================================================================
; 	подпрограмма умножения 2х байтных чисел R4:R3 на  R6:R5
; 	params
;     R4 : R3 * R6 : R5 -> R7:R0:R1:R2
MUL16x16
	clrf  R7
	clrf	R0
	clrf  R1
	movlw	0x80
	movwf	R2

nextbit
	rrf	R4,f
	rrf	R3,f

	btfss	STATUS, C
	goto	nobit_l
	movf	R5,w
	addwf	R1,f

	movf	R6, w
	btfsc	STATUS, C
	incfsz R6, w
	addwf	R0, f
	btfsc	STATUS, C
	incf	R7, f
	bcf	STATUS, C

nobit_l
	btfss	R3, 7
	goto	nobit_h
	movf	R5,w
	addwf	R0,f
	movf	R6, w
	btfsc	STATUS, C
	incfsz R6, w
	addwf	R7, f

nobit_h
	rrf	R7,f
	rrf	R0,f
	rrf	R1,f
	rrf	R2,f

	btfss STATUS, C
	goto	nextbit

	return


;-----------------------------------------------------------------------------
; Умножение.
; Использует R0,R1,R2,R3,R4,R5.

mul_16: ; R1:R0 = R3:R2 * R5:R4
        clrf    R0
        clrf    R1
mul16_Loop:
; R5:R4--;
        decf    R4,f
        btfsc   R4,7
        decf    R5,f
        btfsc   R5,7
        goto    mul16_exit
; R1:R0 += R3:R2.
        movf    R2,0
        addwf   R0,1
        btfsc   STATUS,C
        incf    R1,1   ; Перенос.
        movf    R3,0
        addwf   R1,1
        clrwdt
        goto    mul16_Loop
mul16_exit:
        return
;-----------------------------------------------------------------------------
; Деление.
; Использует R0,R1,R2,R3,R4,R5.

div_16: ; R1:R0 = R3:R2 / R5:R4
; В R3:R2 ост ток.
        clrf    R0
        clrf    R1
        movf    R4,w
        btfss   STATUS,Z
        goto    div16_Loop
        movf    R5,w
        btfss   STATUS,Z
        goto    div16_Loop
        return
div16_Loop:
; Вычит ем RR5:RR4 из RR3:RR2.
        movf    R4,w
        subwf   R2,f     ; F = F - W
        btfss   STATUS,C
        decf    R3,f   ; Перенос.
        btfsc   R3,7
        goto    div16_exit
        movf    R5,w
        subwf   R3,f
        btfss   STATUS,C
        goto    div16_exit
; Увеличиваем частное на  1.
        movlw   1
        addwf   R0,f
        btfsc   STATUS,C
        incf    R1,f
        goto    div16_Loop
; Выход.
div16_exit:
        movf    R4,w
        addwf   R2,f
;        btfss   STATUS,C
        incf    R3,f   ; Перенос.
        movf    R5,w
        addwf   R3,f
        return

;==============================================================================================
LCD_TEMP
	movwf	LCD_PLACE
	
	movlw	0x88
	subwf	R3,w
	btfss	STATUS,Z
	goto	_lcd_temp1
	movlw	0x88
	subwf	R4,w
	btfss	STATUS,Z
	goto	_lcd_temp1

	; print '----'
	movlw	_MINUS
	movwf	LCD_LCD
	clrf	PCLATH
	call 	LCD		; -
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	incf	LCD_PLACE,f
	call	LCD_SPACE	; затираем последний слева
	return
	
_lcd_temp1
	clrf	R5
	call	LCD_ZNAK

	incf	LCD_PLACE,f
	call    B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2

	prv_wi	R1
	bsf	ZERO
	call 	ZERO_

	btfsc	ZERO
	decf	LCD_PLACE,f
	prv_w	R1
	call 	ZERO_

	btfsc	ZERO
	decf	LCD_PLACE,f
	prv_wi	R2
	call 	LCD

	movlw	_POINT		; запятая
	movwf	LCD_LCD
	call 	LCD_INCPLACE

	prv_w	R2
	call 	LCD_INCPLACE

_lcd_temp2
	incf	LCD_PLACE,f
	call	LCD_SPACE	; затираем последний слева

	return

;==============================================================================================
LCD_TEMP2		; вывод температуры для двигателя без знака и дробной части
	movwf	LCD_PLACE
	
	movlw	0x88
	subwf	R3,w
	btfss	STATUS,Z
	goto	_lcd_temp2_1
	movlw	0x88
	subwf	R4,w
	btfss	STATUS,Z
	goto	_lcd_temp2_1

	; print '----'
	incf	LCD_PLACE
	incf	LCD_PLACE
	movlw	_MINUS
	movwf	LCD_LCD
	clrf	PCLATH
	call 	LCD		; -
	call 	LCD_INCPLACE	; -
	return

_lcd_temp2_1
	clrf	PCLATH		; страница памяти 0
	clrf	R5
	incf	LCD_PLACE,f
	call    B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2
	prv_wi	R1
	bsf	ZERO
	call 	ZERO_

;	btfsc	ZERO
;	decf	LCD_PLACE,f
	prv_w	R1
	call 	ZERO_

	btfsc	ZERO
	decf	LCD_PLACE,f
	prv_wi	R2
	call 	LCD

	return

;=====================================================================================
LCD_ZNAK 		; показываем знак температуры
	btfss	R4,7
	goto	lcd_plus
	comf	R4,f
	comf	R3,f
	incf	R3,f
	btfsc	STATUS,Z
	incf	R4,f
	movlw	_MINUS		; минус
	movwf	LCD_LCD
	clrf	PCLATH		; страница памяти 0
	call 	LCD
	return

lcd_plus
	movlw	_PLUS		; плюс
	movwf	LCD_LCD
	clrf	PCLATH		; страница памяти 0
	call 	LCD
	return

;==============================================================================================
LCD_TAHO	; показания тахометра 4 знака хххх
;	clrf	PCLATH		; страница памяти 0
	movwf	LCD_PLACE
	clrf	TAHO_H
	clrf	TAHO_L

	bsf		TAHO_FL2
	btfsc	TAHO_FL2
	btfsc	SEC2_OK
	goto	$+2
	goto	$-3

	bcf		T2CON,TMR2ON	; выкл. отсчет интервала

	btfsc	SEC2_OK	; если знак установлен значит превышен интервал измерения
	goto	lcd_minus	; форсунки отключены, измерение невоможно, выводим "---"

	movf	TAHO_L,w
	movwf	R4
	movf	TAHO_H,w
	movwf	R5

;	timer2 - 80 мкс
;	(60000000/(80*count)) или (60000000/(80*count) * 2) (для парного впрыска)

;	timer2 - 10 мкс
;	(60000000/(10*count)) или (60000000/(10*count) * 2) (для парного впрыска)

	btfsc	ZUMER_ON,5
	goto	taho1			; для парного впрыска

	MOVR7R0R1R2 .12000000 / TAHO_ROUND
	goto taho2
taho1
	MOVR7R0R1R2 .6000000  / TAHO_ROUND

taho2
	call	div32_16	; R7:R0:R1:R2 = R7:R0:R1:R2 / R5:R4

if TAHO_ROUND != 1
	movf	R2,w
	movwf	R4
	movf	R1,w
	movwf	R5

	movlw	TAHO_ROUND
	movwf	R3
	call	MUL16x8 	; R0:R1:R2 = R5:R4 * R3
endif

	movf	R2,w
	movwf	R3
	movf	R1,w
	movwf	R4
	clrf	R5
	bcf	PCLATH,3	; страница памяти 0
	call  B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2
	prv_wi R1
	bsf	ZERO
	call 	ZERO_

	prv_w	R1
	call 	ZERO_

	prv_wi R2
	call 	LCD

	prv_w	R2
	call 	LCD_INCPLACE

#ifdef USE_CUSTOM_CHARS
	movlw	_omin0
	call	LCD_
	movlw	_omin1
	call	LCD_
	movlw	_SPACE
	call	LCD_
	call	LCD_INCPLACE
#else
	movlw	.30
	movwf	PCLATH			; страница памяти 3
	call 	SLOVO21			; об/м
#endif
	retlw	0

lcd_minus
	movlw	_MINUS
	movwf	LCD_LCD
	clrf	PCLATH	; страница памяти 0
	call 	LCD		; -
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	bcf	SEC2_OK
	retlw	0
;==============================================================================================
LCD_VOLT_MIN; показания вольтметра (мин)
	movwf	LCD_PLACE

	movlw	voltHmin
	goto	_lcd_volt

LCD_VOLT_MAX; показания вольтметра (мин)
	movwf	LCD_PLACE

	movlw	voltHmax
	goto	_lcd_volt

LCD_VOLT	; показания вольтметра (макс)
	movwf	LCD_PLACE

	movlw	voltH
	;goto	_lcd_volt

_lcd_volt
	movwf	FSR
	movf	INDF,w	; перенос значений из voltH,L в R5,R4
	movwf	R5
	incf	FSR,f
	movf	INDF,w
	movwf	R4

	;movlw	.5			; 30 / 6 измерений
	movlw	.30
	movwf	R3
	call	MUL16x8 	; R0:R1:R2 = R5:R4 * R3

	; константа для вольтметра
	BANK1
	movf	VCC_CONSTANT,w		; для рез. в делителе 3,3К
	BANK0
	movwf	R4
	clrf	R5
	call	div24_16	; R0:R1:R2 = R0:R1:R2 / R5:R4

	movf	R2,w
	movwf	R3
	movf	R1,w
	movwf	R4
	movf	R0,w
	movwf	R5

	bcf	PCLATH,3	; страница памяти 0
	call  B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2

	call	LCD_SPACE ; затираем один символ слева

	prv_wi R1
	bsf	ZERO
	call 	ZERO_

	prv_w	R1
	call 	ZERO_

	prv_wi R2
	call 	LCD

	movlw	_POINT		;запятая
	call 	LCD_

	prv_w	R2
	call 	LCD_INCPLACE

	movlw	_V
	call 	LCD_

	return


;==============================================================================================
LCD_TIME_COUNTER_LA
	bsf	skipSpaces
	goto	_lcd_time_counter
LCD_TIME_COUNTER
	bcf	skipSpaces
_lcd_time_counter
	movwf	LCD_PLACE

	FSR_ABC	time_H,time_H_B,time_H_C

	movf	INDF,w
	movwf	R0
	incf	FSR,f
	movf	INDF,w
	movwf	R1
	incf	FSR,f
	movf	INDF,w
	movwf	R2

	clrf	R5
	movlw	.30
	movwf	R4
	call	div24_16	; R0:R1:R2 = R0:R1:R2 / R5:R4 (время в минутах)

	clrf	R5
	movlw	.60
	movwf	R4
	call	div24_16	; R0:R1:R2 (часы) = R0:R1:R2 / R5:R4 (минуты R6:R7 - остаток)

	clrf	PCLATH		; страница памяти 0

	movf	R7,w
	call	BinBCD		; R0 - минуты в BCD
	movf	R0,w
	movwf	R8

	clrf	R5
	movf	R1,w
	movwf	R4
	movf	R2,w
	movwf	R3

	call	B2_BCD		; R5:R4:R3 => BCD R0:R1:R2

	bsf	ZERO
	prv_w	R1
	bsf	PCLATH,3
	btfss	skipSpaces
	goto	_noskip01
	xorlw	0x30
	btfsc	STATUS,Z
	goto	$+4				; skip
	bcf	skipSpaces
_noskip01
	clrf	PCLATH
	call 	ZERO_

	prv_wi	R2
	bsf	PCLATH,3
	btfss	skipSpaces
	goto	_noskip02
	xorlw	0x30
	btfsc	STATUS,Z
	goto	$+4				; skip
	bcf	skipSpaces
_noskip02
	clrf	PCLATH
	call 	ZERO_

	prv_w	R2
	clrf	PCLATH
	call 	LCD

	movlw	0x3A		; ':'
	call 	LCD_

	prv_wi	R8
	call 	LCD_INCPLACE

	prv_w	R8
	call 	LCD_INCPLACE
	return

;==============================================================================================
LCD_FUEL			; показываем общий расход 5 знаков ххх.х
	clrf	PCLATH		; страница памяти 0
	movwf	LCD_PLACE

	FSR_ABC	FUEL_00H,FUEL_00H_B,FUEL_00H_C
	movf	INDF,w
	movwf	R4
	incf	FSR,f
	movf	INDF,w
	movwf	R3
	clrf	R5
	call  B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2

	prv_w	R0
	bsf	ZERO
	call 	ZERO_

	prv_wi	R1
	call 	ZERO_

	prv_w	R1
	call 	LCD

	movlw	_POINT		;запятая
	call 	LCD_

	prv_wi	R2
	call 	LCD_INCPLACE

#ifdef LCD_ENGLISH
	movlw	_l
#else
	movlw	л
#endif
	btfss	ZUMER_ON,6
	movlw	_l
	call 	LCD_

	return

;==============================================================================================
LCD_FUEL2
; показываем мгновенный расход 4 знака хх.х
; расход приводим к литрам: FUEL_TMP3,4/(65*FUEL_CONST)
; пробег приводим к 100 км: KMH_L,H/(100*ODO_CON1L,H)
	movwf	LCD_PLACE

	movf	ODO_CON4H,w
	movwf	R4
	movf	ODO_CON4L,w
	movwf	R3

	movf	FUEL_TMP3,w
	movwf	R5
	movf	FUEL_TMP4,w
	movwf	R6
	call	MUL16x16	; R4 : R3 * R6 : R5 -> R7:R0:R1:R2

	movf	KMH_L,w
	movwf	R4
	movf	KMH_H,w
	movwf	R5
	call	div32_16	; R7:R0:R1:R2 = R7:R0:R1:R2 / R5:R4

	movf	FUEL_CONST,w
	movwf	R4
	clrf	R5
	call	div24_16	; R0:R1:R2 = R0:R1:R2 / R5:R4

	movf	R2,w
	movwf	R3
	movf	R1,w
	movwf	R4
	clrf	R5
	bcf	PCLATH,3	; страница памяти 0
	call  B2_BCD          ; After conversion the Decimal Number
                            ; in R0,R1,R2

#ifdef USE_CUSTOM_CHARS
	movlw	_SPACE
	call	LCD_
#endif

	prv_w	R1
	bsf		ZERO
	call 	ZERO_

	prv_wi	R2
	call 	LCD

	movlw	_POINT		;запятая
	call 	LCD_

	prv_w	R2
	call 	LCD_INCPLACE

#ifdef USE_CUSTOM_CHARS
	movlw	_lkm0
	call	LCD_
	movlw	_lkm1
	call	LCD_
#else
	movlw	.30
	movwf	PCLATH			; страница памяти 3
	call 	SLOVO23			;л/10
#endif
	return

;==============================================================================================
LCD_FUEL3
;  показываем часовой расход 3 знака х.х
;  приближенная формула = (расход за 2 сек * 28)/константа

	movwf	LCD_PLACE

	movf	FUEL_TMP3,w
	movwf	R4
	movf	FUEL_TMP4,w
	movwf	R5
	movlw	.28
	btfsc	ZUMER_ON,5
	movlw	.14 		; для парного впрыска
	movwf	R3
	call	MUL16x8 	; R0:R1:R2 = R5:R4 * R3

	movf	FUEL_CONST,w
	movwf	R4
	clrf	R5
	call	div24_16	; R0:R1:R2 = R0:R1:R2 / R5:R4

	movf	R2,w
	movwf	R3
	movf	R1,w
	movwf	R4
	clrf	R5
	bcf	PCLATH,3	; страница памяти 0
	call    B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2
#ifdef USE_CUSTOM_CHARS
	movlw	_SPACE
	call	LCD_
#endif

	prv_wi	R1
	bsf	ZERO
	call 	ZERO_

	prv_w	R1
	call 	LCD

	movlw	_POINT		;запятая
	call 	LCD_

	prv_wi	R2
	call 	LCD_INCPLACE

#ifdef USE_CUSTOM_CHARS
	movlw	_lh0
	call	LCD_
	movlw	_lh1
	call	LCD_
#else
	movlw	.30
	movwf	PCLATH			; страница памяти 3
	call 	SLOVO22			; л/ч
#endif

	return

;==============================================================================================
LCD_FUEL4
; показываем средний расход 4 знака хх.х
; умножаем расход на 100

	movwf	LCD_PLACE

#ifdef AVERAGE_MIN_FUEL
	FSR_ABC FUEL_00L,FUEL_00L_B,FUEL_00L_C

	movlw	low AVERAGE_MIN_FUEL
	subwf   INDF,w

	movlw   high AVERAGE_MIN_FUEL
	btfss   STATUS,C
	addlw   .1
	decf	FSR,f
	subwf   INDF,w

	btfss   STATUS,C
	goto	lcd_fuel4_1 ; '----', если общий расход меньше заданного
#endif

	; get and save odo00
	call	get_odo00

#ifdef AVERAGE_MIN_DIST

	movlw	low AVERAGE_MIN_DIST
	subwf   R1,w

	movlw   high AVERAGE_MIN_DIST
	btfss   STATUS,C
	addlw   .1
	subwf   R2,w

	btfss   STATUS,C
	goto	lcd_fuel4_1 ; '----', если общий расход меньше заданного
#endif

	movf	R0,w
	movwf	R13
	movf	R1,w
	movwf	R14
	movf	R2,w
	movwf	R15

	FSR_ABC	FUEL_00H,FUEL_00H_B,FUEL_00H_C

	movf	INDF,w
	movwf	R5
	incf	FSR,f
	movf	INDF,w
	movwf	R4
	movlw	.100
	movwf	R3
	call	MUL16x8 	; R0:R1:R2 = R5:R4 * R3

	movf	R0,w
	movwf	R3
	movf	R1,w
	movwf	R4
	movf	R2,w
	movwf	R5
	clrf	R0
	clrf	R1
	clrf	R2
	
; делим на пробег
	call	div_48by24	; R0:R1:R2:R3:R4:R5 = R0:R1:R2 / R5:R4
	iorlw	0x00
	btfsc	STATUS,Z
	goto	lcd_fuel4_1

	; reorder R3:R4:R5 -> R5:R4:R3 (swap R3 and R5)
	MOVF  R3,W ; W:=A
	XORWF R5,W ; W:=A^B
	XORWF R3,F ; X:=((A^B)^A)=B
	XORWF R5,F ; Y:=((A^B)^B)=A

	bcf	PCLATH,3	; страница памяти 0
	call	B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2

#ifdef USE_CUSTOM_CHARS
	movlw	_SPACE
	call	LCD_
#endif

	prv_w	R1
	bsf	ZERO
	call 	ZERO_

	prv_wi	R2
	call 	LCD

	movlw	_POINT		;запятая
	call 	LCD_

	prv_w	R2
	call 	LCD_INCPLACE

lcd_fuel4_2
#ifdef USE_CUSTOM_CHARS
	pageselw LCD_
	movlw	_lkm0
	call	LCD_
	movlw	_lkm1
	call	LCD_
#else
	movlw	.30
	movwf	PCLATH			; страница памяти 3
	call 	SLOVO23			;л/10
#endif

	return

lcd_fuel4_1
	clrf	PCLATH	; страница памяти 0
#ifdef USE_CUSTOM_CHARS
	movlw	_SPACE
	call	LCD_
#endif
	movlw	0x2D
	movwf	LCD_LCD
	call 	LCD		; -
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	call 	LCD_INCPLACE	; -
	gotop	lcd_fuel4_2

	; суточный счетчик с разрешением 100 м
get_odo00
	; R0:R1:R2 = odo_00 * 10 + (((odo00temp * 10) / pulses per km (ODO_CON1)) % 10)

	; R0:R1:R2 = odo00temp * 10
	FSR_ABC	odo_00Htemp,odo_00Htemp_B,odo_00Htemp_C

	movf	INDF,w
	movwf	R5
	incf	FSR,f
	movf	INDF,w
	movwf	R4
	movlw	.10
	movwf	R3
	call	MUL16x8 	; R0:R1:R2 = R5:R4 * R3

	; R0:R1:R2 = (odo00temp * 10) / pulses per km (ODO_CON1)
	movf	ODO_CON1H,w
	movwf	R5
	movf	ODO_CON1L,w
	movwf	R4
	call	div24_16	; R0:R1:R2 = R0:R1:R2 / R5:R4

	; R6 = R2 = (((odo00temp * 10) / pulses per km (ODO_CON1)) % 10)
	movf	R2,w
	movwf	R6

	; R0:R1:R2 = odo_00 * 10
	FSR_ABC	odo_00H,odo_00H_B,odo_00H_C

	movf	INDF,w
	movwf	R5
	incf	FSR,f
	movf	INDF,w
	movwf	R4
	movlw	.10
	movwf	R3
	call	MUL16x8 	; R0:R1:R2 = R5:R4 * R3

	;add R6 (0-9) to R0:R1:R2
	movf    R6,w
	addwf   R2,f
	btfsc   STATUS,C
	incf    R1,f   ; Перенос.

	return

;=====================================================================================
	
LCD_ODOM_NO_SKIP
	bcf	skipSpaces
	goto	_lcd_odom_1
LCD_ODOM
	bsf	skipSpaces
_lcd_odom_1
	movwf	LCD_PLACE
	call	get_odo00
_lcd_odom_2
	movf	R0,w
	movwf	R5
	movf	R1,w
	movwf	R4
	movf	R2,w
	movwf	R3

	clrf	PCLATH		; страница памяти 0
	call    B2_BCD          ; After conversion the Decimal Number
;                               ; in R0,R1,R2
	bsf	ZERO

	bsf	PCLATH,3
	prv_w	R0
#ifdef USE_CUSTOM_CHARS
	btfss	skipSpaces
	goto	_noskip11
	xorlw	0x30
	btfsc	STATUS,Z
	goto	$+4				; skip
	bcf	skipSpaces
_noskip11
#endif
	clrf	PCLATH
	call 	ZERO_

	bsf	PCLATH,3
	prv_wi	R1
#ifdef USE_CUSTOM_CHARS
	btfss	skipSpaces
	goto	_noskip12
	xorlw	0x30
	btfsc	STATUS,Z
	goto	$+4				; skip
	bcf	skipSpaces
_noskip12
#endif
	clrf	PCLATH
	call 	ZERO_

	bsf	PCLATH,3
	prv_w	R1
#ifdef USE_CUSTOM_CHARS
	btfss	skipSpaces
	goto	_noskip13
	xorlw	0x30
	btfsc	STATUS,Z
	goto	$+4				; skip
	bcf		skipSpaces
_noskip13
#endif
	clrf	PCLATH
	call 	ZERO_

	clrf	PCLATH
	prv_wi	R2
	call 	LCD

	movlw	_POINT		;запятая
	movwf	LCD_LCD
	call 	LCD_INCPLACE

	prv_w	R2
	call 	LCD_INCPLACE

	movlw	.31
	movwf	PCLATH			; страница памяти 3
	call	SLOVO20		; км

	return

;=====================================================================================
LCD_ODOM2_TO
	movwf	FSR
	clrf	R5
	movf	INDF,w
	movwf	R4
	incf	FSR,f
	movf	INDF,w
	movwf	R3
	movlw	0xBF
	goto	LCD_ODOM2

LCD_ODOM2_MAIN
	movwf	LCD_PLACE
	movf	odo_01L,w
	movwf	R3
	movf	odo_01F,w
	movwf	R4
	movf	odo_01H,w
	movwf	R5
	goto	LCD_ODOM2_1

LCD_ODOM2
	movwf	LCD_PLACE
LCD_ODOM2_1
	clrf	PCLATH		; страница памяти 0
	call    B2_BCD          ; After conversion the Decimal Number
;                               ; in R0,R1,R2

	bsf	ZERO
	prv_wi	R0
	call 	ZERO_

	prv_w	R0
	call 	ZERO_

	prv_wi	R1
	call 	ZERO_

	prv_w	R1
	call 	ZERO_

	prv_wi	R2
	call 	ZERO_

	prv_w	R2
	call 	LCD

	movlw	.31
	movwf	PCLATH			; страница памяти 3
	call	SLOVO20			; км

	return

LCD_MH
	movwf	LCD_PLACE
	clrf	PCLATH		; страница памяти 0
	call    B2_BCD          ; After conversion the Decimal Number
;                               ; in R0,R1,R2

	bsf	ZERO
	prv_wi	R1
	call 	ZERO_

	prv_w	R1
	call 	ZERO_

	prv_wi	R2
	call 	ZERO_

	prv_w	R2
	call 	LCD

	movlw	.30
	movwf	PCLATH			; страница памяти 3
	call	SLOVO20_1		; м/ч

	return


;==============================================================================================

; максимальная скорость с момента включения
LCD_KMH_MAX
	bsf	skipSpaces
	movwf	LCD_PLACE
;	move from speedHmax:speedLmax to R4:R3
	movlw	speedHmax
	movwf	FSR
	goto	_lcd_kmh2

LCD_KMH_LA
	bsf	skipSpaces
	addlw	-1
	goto	$+2
; текущая скорость
LCD_KMH
; формула KMH_(H,L)*18000/ODO_CON1(H,L) 7 знаков ХХХкм/ч
; скорость в speedH:speedL
	bcf	skipSpaces
	movwf	LCD_PLACE
_lcd_kmh

;	move from speedH:speedL to R4:R3
	movlw	speedH
	movwf	FSR
_lcd_kmh2
	movf	INDF,w
	movwf	R4
	incf	FSR,f
	movf	INDF,w
	movwf	R3
	clrf	R5

#ifdef USE_CUSTOM_CHARS
	btfsc	skipSpaces
	goto	_lcd_kmh_skip_spaces
	bcf	PCLATH,3	; страница памяти 0
	movlw	_SPACE
	movwf	LCD_LCD
	call	LCD
	call	LCD_INCPLACE
	incf	LCD_PLACE,f
_lcd_kmh_skip_spaces
#endif

	bcf	PCLATH,3	; страница памяти 0
	call    B2_BCD       ; After conversion the Decimal Number
                         ; in R0,R1,R2

	call	LCD_SPACE ; затираем один символ слева

	prv_wi	R1
	bsf		ZERO
	call 	ZERO_

	prv_w	R1
	call 	ZERO_

	prv_wi	R2
	call 	LCD
#ifdef USE_CUSTOM_CHARS
	movlw	_kmh0
	call	LCD_
	movlw	_kmh1
	call	LCD_
#else
	movlw	.31
	movwf	PCLATH			; страница памяти 3
	call	SLOVO19		; км/ч
#endif

	return

;=====================================================================================
LCD_TIME
	movwf	LCD_PLACE
	btfsc	time_OK
	goto	T_OK
	clrf	PCLATH		; страница памяти 0

	movlw	0x2D
	call 	LCD_		; -
	movlw	0x2D
	call 	LCD_		; -
	movlw	0x3A
	call 	LCD_		; двоеточие
	movlw	0x2D
	call 	LCD_		; -
	movlw	0x2D
	call 	LCD_		; -

	return

T_OK
	clrf	PCLATH		; страница памяти 0
	movlw 0x80
	movwf	LCD_PLACE

	swapf	HOURS,w
	andlw	b'00000011'
	iorlw	b'00110000'
	movwf	LCD_LCD
	call 	LCD		; десятки часов

	prv_w	HOURS
	call 	LCD_INCPLACE	; единицы часов


	movlw	0x3A
	call 	LCD_		; двоеточие

	prv_wi MINUTES
	call 	LCD_INCPLACE	; десятки минут

	prv_w	MINUTES
	call 	LCD_INCPLACE	; единицы минут

	return

;=====================================================================================
; подпрограмма замера разгона до 100кмч
;=====================================================================================
; регистры fTEMP - лимит времени 25 сек.
;	  TAHO_H,L - текущая скорость
;       fCRC - время разгона (дискретность 0.1с)
; флаги FL100 - флаг готовности измерения
;       FL100_in - флаг измерения

SPEED100
	btfsc	DRIVE_FL
	goto	sp_end
	movlw	0x64			; лимит ожидания 10 сек.
	movwf	fTEMP
	clrf	fCRC

; расчитаем значение TAHO_H,L при 100кмч
; (36000000 / timer2 period(мкс) ) / ODO_CON1(H,L)

	MOVR0R1R2 .3600000

	movf	ODO_CON1L,w
	movwf	R4
	movf	ODO_CON1H,w
	movwf	R5
	call	div24_16	; R0:R1:R2 = R0:R1:R2 / R5:R4

	clrf	PCLATH
	call	LCD_CLEAR
	movlw	.30
	movwf	PCLATH		; страница памяти 3
	call	SLOVO24		; ожидание старта
	bsf	PCLATH,3		; страница памяти 1

sp4	movf	fTEMP,w
	btfsc	STATUS,Z
	goto	sp_end
	btfss	DRIVE_FL
	goto 	sp4

	movlw	0xC8			; лимит разгона 20 сек.
	movwf	fTEMP
	clrf	PCLATH
	call	LCD_CLEAR
	movlw	.30
	movwf	PCLATH		; страница памяти 3
	call	SLOVO9		; измерение
	bsf	PCLATH,3		; страница памяти 1


sp1	clrf	TAHO_H
	clrf	TAHO_L
	bsf	FL100

sp2	movf	fTEMP,w
	btfsc	STATUS,Z
	goto	sp_end
	btfsc	FL100
	goto 	sp2

; процедура сравнениия
; если TAHO_H:TAHO_L<=R1:R2 то достигли скорости 100кмч
	movf	TAHO_H,w
	subwf R1,w
	btfss STATUS,C
	goto sp1
  	movfw TAHO_L
  	subwf R2,w
	btfss STATUS,C
  	goto sp1

	movlw	0x3C ; таймер на 6 сек.
	movwf	fTEMP
	bsf	FL_zumer
	bsf	FL_zumerВ3
	movf	fCRC,w
	movwf	R3
	clrf	R4
	clrf	R5
	clrf	PCLATH
	call	LCD_CLEAR
	movlw	0xC4
	movwf	LCD_PLACE
	call    B2_BCD       ; After conversion the Decimal Number
                         ; in R0,R1,R2

	prv_w	R1
	call 	LCD_INCPLACE
	prv_wi	R2
	call 	LCD_INCPLACE
	movlw	_POINT		;запятая
	movwf	LCD_LCD
	call 	LCD_INCPLACE
	prv_w	R2
	call 	LCD_INCPLACE

	movlw	.31
	movwf	PCLATH		; страница памяти 3
	call	SLOVO26		; сек.
	movlw	.30
	movwf	PCLATH		; страница памяти 3
	call	SLOVO25		; результат
	bsf	PCLATH,3		; страница памяти 1

sp3	movf	fTEMP,w
	btfss	STATUS,Z
	goto	sp3

sp_end
	clrf	PCLATH
	clrf	menu_position
	bcf	button_OK1
	bcf	button_OK2
	bsf	SEC2_OK
	bcf	FL100
	bsf	FL_zumer
	bsf	FL_zumerВ1
	call	LCD_CLEAR
	goto	main3


; ===============================================================================
;	32 / 16 = 32
#define divid0 R2
#define divid1 R1
#define divid2 R0
#define divid3 R7

#define remdrH R9
#define remdrL R8

#define divisH R5
#define divisL R4

#define bitcnt R3

div32_16
	   movlw .32      ; 32-bit divide by 16-bit
       movwf bitcnt
       clrf remdrH   ; Clear remainder
       clrf remdrL

dvloop clrc          ; Set quotient bit to 0
                     ; Shift left dividend and quotient
       rlf divid0    ; lsb
       rlf divid1
       rlf divid2
       rlf divid3    ; lsb into carry
       rlf remdrL    ; and then into partial remainder
       rlf remdrH

       skpnc         ; Check for overflow
       goto subd
       movfw divisH  ; Compare partial remainder and divisor
       subwf remdrH,w
       skpz
       goto testgt   ; Not equal so test if remdrH is greater
       movfw divisL  ; High bytes are equal, compare low bytes
       subwf remdrL,w
testgt skpc          ; Carry set if remdr >= divis
       goto remrlt

subd   movfw divisL  ; Subtract divisor from partial remainder
       subwf remdrL
       skpc          ; Test for borrow

       decf remdrH   ; Subtract borrow
       movfw divisH
       subwf remdrH
       bsf divid0,0  ; Set quotient bit to 1
                     ; Quotient replaces dividend which is lost
remrlt decfsz bitcnt
       goto dvloop
       return

mul24_24:
#define Product	R0
#define Multipland R13
#define BitCount R7
;24 x 24 Multiplication
;Input:
; Multiplier - 3 bytes (shared with Product) R3:R4:R5
; Multiplicand - 3 bytes (not modified)	     R13:R14:R15
;Temporary:
; Bitcount
;Output:
; Product - 6 bytes R0:R1:R2:R3:R4:R5

        CLRF    Product         ; clear destination
        CLRF    Product+1
        CLRF    Product+2

        
        MOVLW   D'24'
        MOVWF   BitCount        ; number of bits

        RRF     Product+3,F     ; shift out to carry
        RRF     Product+4,F     ; next multiplier bit
        RRF     Product+5,F

ADD_LOOP_24x24

        BTFSS   STATUS,C        ; if carry is set we must add multipland
                                ; to the product
          GOTO  SKIP_LOOP_24x24 ; nope, skip this bit
                
        MOVF    Multipland+2,W  ; get LSB of multiplicand
        ADDWF   Product+2,F     ; add it to the lsb of the product
  
        MOVF    Multipland+1,W  ; middle byte
        BTFSC   STATUS,C        ; check carry for overflow
        INCFSZ  Multipland+1,W  ; if carry set we add one to the source 
        ADDWF   Product+1,F     ; and add it  (if not zero, in
                                ; that case mulitpland = 0xff->0x00 )
        
        MOVF    Multipland,W    ; MSB byte
        BTFSC   STATUS,C        ; check carry
        INCFSZ  Multipland,W
        ADDWF   Product,F       ; handle overflow

SKIP_LOOP_24x24
        ; note carry contains most significant bit of
        ; addition here

        ; shift in carry and shift out
        ; next multiplier bit, starting from less
        ; significant bit

        RRF     Product,F
        RRF     Product+1,F
        RRF     Product+2,F
        RRF     Product+3,F
        RRF     Product+4,F
        RRF     Product+5,F

        DECFSZ  BitCount,F
        GOTO    ADD_LOOP_24x24
        RETURN

div_48by24:
;   divide R0:R1:R2:R3:R4:R5 by R13:R14:R15
;   use R7, R8-R10
#define	Divisor R13
#define Dividend R0
#define Temp	fTEMPER_L
#define Temp2	R7
#ifdef OBSOLETE    
        ; Test for zero division
        MOVF    Divisor,W
        IORWF   Divisor+1,W
        IORWF   Divisor+2,W
        BTFSC   STATUS,Z
        RETLW   0x00    ; divisor = zero, not possible to calculate return with zero in w

        ; prepare used variables
        CLRF    Temp
        CLRF    Temp+1
        CLRF    Temp+2

        clrf    Temp2

        MOVLW   D'48'           ; initialize bit counter
        MOVWF   BitCount

DIVIDE_LOOP_48by24
        RLF     Dividend+5,F
        RLF     Dividend+4,F
        RLF     Dividend+3,F
        RLF     Dividend+2,F
        RLF     Dividend+1,F
        RLF     Dividend,F
        ; shift in highest bit from dividend through carry in temp
        RLF     Temp+2,F
        RLF     Temp+1,F
        RLF     Temp,F

        rlf     Temp2, f

        MOVF    Divisor+2,W     ; get LSB of divisor
        btfsc   Temp2, 7
        goto    Div48by24_add

        ; subtract 24 bit divisor from 24 bit temp
        SUBWF   Temp+2,F        ; subtract

        MOVF    Divisor+1,W     ; get middle byte
        SKPC                    ;  if overflow ( from prev. subtraction )
        INCFSZ  Divisor+1,W     ; incresase source
        SUBWF   Temp+1,F        ; and subtract from dest.

        MOVF    Divisor,W       ; get top byte
        SKPC                    ;  if overflow ( from prev. subtraction )
        INCFSZ  Divisor,W       ; increase source
        SUBWF   Temp,F          ; and subtract from dest.

        movlw 1
        skpc
        subwf   Temp2, f
        GOTO    DIVIDE_SKIP_48by24 ; carry was set, subtraction ok, continue with next bit

Div48by24_add
        ; result of subtraction was negative restore temp
        ADDWF   Temp+2,F        ; add it to the lsb of temp

        MOVF    Divisor+1,W     ; middle byte
        BTFSC   STATUS,C        ; check carry for overflow from previous addition
        INCFSZ  Divisor+1,W     ; if carry set we add 1 to the source
        ADDWF   Temp+1,F        ; and add it if not zero in that case Product+Multipland=Product

        MOVF    Divisor,W       ; MSB byte
        BTFSC   STATUS,C        ; check carry for overflow from previous addition
        INCFSZ  Divisor,W
        ADDWF   Temp,F          ; handle overflow

        movlw 1
        skpnc
        addwf   Temp2, f

DIVIDE_SKIP_48by24
        DECFSZ  BitCount,F      ; decrement loop counter
        GOTO    DIVIDE_LOOP_48by24      ; another run
        ; finally shift in the last carry
        RLF     Dividend+5,F
        RLF     Dividend+4,F
        RLF     Dividend+3,F
        RLF     Dividend+2,F
        RLF     Dividend+1,F
        RLF     Dividend,F
        RETLW   0x01    ; done
#else
	;****************************************************
;max time in loop: 26 cycles
DIVIDE_48by23
        ; Test for zero division
        MOVF    Divisor,W
        IORWF   Divisor+1,W
        IORWF   Divisor+2,W
        BTFSC   STATUS,Z
        RETLW   0x00    ; divisor = zero, not possible to calculate return with zero in w

        ; prepare used variables
        CLRF    Temp
        CLRF    Temp+1
        CLRF    Temp+2

        MOVLW   D'48'           ; initialize bit counter
        MOVWF   BitCount

        setc
DIVIDE_LOOP_48by23
        RLF     Dividend+5,F
        RLF     Dividend+4,F
        RLF     Dividend+3,F
        RLF     Dividend+2,F
        RLF     Dividend+1,F
        RLF     Dividend,F
        ; shift in highest bit from dividend through carry in temp
        RLF     Temp+2,F
        RLF     Temp+1,F
        RLF     Temp,F

        MOVF    Divisor+2,W     ; get LSB of divisor
        btfss   Dividend+5, 0
        goto    Div48by23_add
        
        ; subtract 23 bit divisor from 24 bit temp 
        SUBWF   Temp+2,F        ; subtract

        MOVF    Divisor+1,W     ; get middle byte
        SKPC                    ;  if overflow ( from prev. subtraction )
        INCFSZ  Divisor+1,W     ; incresase source 
        SUBWF   Temp+1,F        ; and subtract from dest.
        
        MOVF    Divisor,W       ; get top byte
        SKPC                    ;  if overflow ( from prev. subtraction )
        INCFSZ  Divisor,W       ; increase source 
        SUBWF   Temp,F          ; and subtract from dest.
        GOTO    DIVIDE_SKIP_48by23 ; carry was set, subtraction ok, continue with next bit

Div48by23_add
        ; result of subtraction was negative restore temp
        ADDWF   Temp+2,F        ; add it to the lsb of temp
  
        MOVF    Divisor+1,W     ; middle byte
        BTFSC   STATUS,C        ; check carry for overflow from previous addition
        INCFSZ  Divisor+1,W     ; if carry set we add 1 to the source
        ADDWF   Temp+1,F        ; and add it if not zero in that case Product+Multipland=Product
        
        MOVF    Divisor,W       ; MSB byte
        BTFSC   STATUS,C        ; check carry for overflow from previous addition
        INCFSZ  Divisor,W
        ADDWF   Temp,F          ; handle overflow

DIVIDE_SKIP_48by23
        DECFSZ  BitCount,F      ; decrement loop counter
        GOTO    DIVIDE_LOOP_48by23      ; another run
        ; finally shift in the last carry
        RLF     Dividend+5,F
        RLF     Dividend+4,F
        RLF     Dividend+3,F
        RLF     Dividend+2,F
        RLF     Dividend+1,F
        RLF     Dividend,F
        RETLW   0x01    ; done
#endif
;****************************************************
	
_CLEAR:
	clrf	odo_00L
	clrf	odo_00H

	clrf	FUEL_00H
	clrf	FUEL_00L

	BANK1
	clrf	time_H
	clrf	time_F
	clrf	time_L
	BANK0

	movlw	.65			; первый счетчик считает до 65
	btfsc	ZUMER_ON,5
	movlw	.130			; для парного впрыска больше в 2 раза
	movwf	FUEL_TMP1
	clrf	FUEL_TMP2

	clrf	odo_00Ltemp
	clrf	odo_00Htemp

	return

_CLEAR_B:
	BANK1
	clrf	odo_00L_B
	clrf	odo_00H_B

	clrf	FUEL_00H_B
	clrf	FUEL_00L_B

	clrf	time_H_B
	clrf	time_F_B
	clrf	time_L_B
	BANK0
	movlw	.65			; первый счетчик считает до 65
	btfsc	ZUMER_ON,5
	movlw	.130			; для парного впрыска больше в 2 раза
	BANK1
	movwf	FUEL_TMP1_B
	clrf	FUEL_TMP2_B

	clrf	odo_00Ltemp_B
	clrf	odo_00Htemp_B
	BANK0
	return

_CLEAR_C:
	BANK1
	clrf	odo_00L_C
	clrf	odo_00H_C

	clrf	FUEL_00H_C
	clrf	FUEL_00L_C

	clrf	time_H_C
	clrf	time_F_C
	clrf	time_L_C
	BANK0
	movlw	.65			; первый счетчик считает до 65
	btfsc	ZUMER_ON,5
	movlw	.130			; для парного впрыска больше в 2 раза
	BANK1
	movwf	FUEL_TMP1_C
	clrf	FUEL_TMP2_C

	clrf	odo_00Ltemp_C
	clrf	odo_00Htemp_C
	BANK0
	return

       org	0x0FB6
;=====================================================================================
; подпрограмма вывода звуковых сигналов
;=====================================================================================
;	исполузуюся регистры:
;	ZUM_H - длительность сигнала в 0,1сек
;	ZUM_L - длительность паузы  0,1сек
;	ZUM_N - 	количество повторов
;	флаги:
;	FL_zumer - флаг возникновения события
;	FL_zumerS - флаг работы зумера
;	FL_zumerВ1, FL_zumerВ2, L_zumerВ3 - источники события
;	FL_zumerH - флаг вывода паузы
;	параметры сигнала для конкретного события загружаются в ZUMER_READ
;	данные берутся из таблицы ZUMER_R (3 значения)

ZUMER_
	btfsc	FL_zumerS	; если первое обращение к процедуре
	goto	Z4

;	загружаем рабочие регистры
	bsf	zumer			; включим сигнал
	movlw	0x75
	movwf	FSR			; здесь адрес регистра в который будем грузить

; задаем смещение для загрузки из таблицы данных
; для различных видов сигналов
	movlw	.0
	btfsc FL_zumerВ2
	movlw	.3
	btfsc FL_zumerВ3
	movlw	.6
	movwf	ZUM_tmp
	bsf	FL_zumerS
Z_UP1
	call 	ZUMER_R
	movwf	INDF			; записываем регистр
	incf 	ZUM_tmp,f
	incf	FSR,f
	movlw	ZUM_L + 0x01
	xorwf	FSR,w
	JZN	Z_UP1

; обработка длительности сигнала
Z4	btfsc	FL_zumerH
	goto	Z1
	decf	ZUM_H,f
	btfss	STATUS,Z
	goto	Z3
	bcf	zumer			; выключим сигнал

; обработка повторов сигнала
	decf	ZUM_N,f
	btfss	STATUS,Z
	goto	Z2
	bcf	FL_zumer
	bcf	FL_zumerS
	bcf	FL_zumerВ1
	bcf	FL_zumerВ2
	bcf	FL_zumerВ3
	goto	Z3
Z2	bsf	FL_zumerH
	goto	Z3

; обработка паузы сигнала
Z1	decf	ZUM_L,f
	btfss	STATUS,Z
	goto	Z3
	bcf	FL_zumerH


	bsf	zumer			; включим сигнал
	movlw	0x76		; ZUM_N не грузим
	movwf	FSR			; здесь адрес регистра в который будем грузить

; задаем смещение для загрузки из таблицы данных
; для различных видов сигналов
	movlw	.1
	btfsc FL_zumerВ2
	movlw	.4
	btfsc FL_zumerВ3
	movlw	.7
	movwf	ZUM_tmp
	bsf		FL_zumerS
Z_UP2
	call 	ZUMER_R
	movwf	INDF			; записываем регистр
	incf 	ZUM_tmp,f
	incf	FSR,f
	movlw	ZUM_L + 0x01
	xorwf	FSR,w
	JZN	Z_UP2


Z3	clrf	PCLATH
	goto	PR_T11

;	из таблицы берутся по три значения
;	1. кол. повторов
;	2. длительность импульса 0,1сек +1
;	3. длительность паузы 0,1сек +1
;	смещение передается параметром ZUM_tmp
ZUMER_R
	movf	ZUM_tmp,w
	addwf	PCL,f
;=== 	1 короткий бип (0.1сек)
	retlw	0x01
	retlw	0x02
	retlw	0x02
;===	5 длинных бипов (0.5сек через 0.1сек)
	retlw	0x05
	retlw	0x06
	retlw	0x02
;===	5 коротких бипов (0.1сек через 0.1сек)
	retlw	0x05
	retlw	0x02
	retlw	0x02
	retlw	0x02


;=====================================================================================
;=====================================================================================
;=====================================================================================
;=================          С Т Р А Н И Ц А  2           =============================
;=====================================================================================
;=====================================================================================
;=====================================================================================
    	org	0x1000	; страница памяти 2

;==============================================================================================
INIT:				; инициализация БК
	clrf	INTCON
	clrf	PORTA
	clrf	fFLAGS_REG1
	clrf	fFLAGS_REG2
	clrf	fFLAGS_REG3

	movlw	b'00110001'	; биты конфигурации TMR1 (prescaler 1:8, timer on)
	movwf	T1CON

	movlw	b'00000000'	; биты конфигурации TMR2 (prescaler 1:1, postscaler 1:1)
	movwf	T2CON

	BANK1
	movlw	b'10111000'
	movwf	OPTION_REG	; TMR0 от внешнего генератора RA4
	movlw	b'00001110'
	movwf	TRISA		; настройка порта А
	movlw	b'11111100'
	movwf	TRISB		; настройка порта B
	clrf	TRISC		; настройка порта С
	movlw	b'00000110'
	movwf	ADCON1		; все порты А цифровые
	bsf	PIE1,TMR1IE	; разрешение прерывания от TMR1
	bsf	PIE1,TMR2IE	; разрешение прерывания от TMR2

	movlw	.49
	movwf	PR2		; установка предделителя для TMR2 (период timer2 = 50 * 200нс = 10мкс)

	BANK0

	MOVLW	H'00'
	MOVWF	PORTB
	MOVLW	b'00001110'
	MOVWF	PORTC

	bsf	SDA
	bsf	SCL

	btfsc	CONTROL_PWR	; если включено зажигание
	bsf	ON_PWR		; удерживаем питание

	clrf	TMR0
	clrf	PIR1

	; clear ram
	bsf	STATUS,IRP
	call	clear_ram	; bank2/3
	bcf	STATUS,IRP
	call	clear_ram	; bank0/1

	clrf	PCLATH
	CALL	LCD_INIT	; инициализация дисплея
	call	LCD_CLEAR
	movlw	0x10
	movwf	PCLATH

	call	READ		; read EEPROM

	; init values
	BANK1
	;init volt_min
	movlw	0x7F
	movwf	voltHmin
	decf	voltLmin,f
	BANK0

	movlw	.20		; инициализация таймера 2 сек.
	movwf	S_fCOUNTER
	bcf	SEC2_OK

	movlw	.1		; первая инициализация таймера 1 сек. (по крайней мере в протеусе с первого раза датчики не читаются)
	movwf	fTIMER

	bsf	INTCON,RBIE	; прерывания от RB6...7
	movf	PORTB,w		; чтение порта ОБЯЗАТЕЛЬНО!
	bcf	INTCON,RBIF	; сбросим флаг прерывания от RB6..7

	bsf	INTCON,PEIE	; разрешаем прерывание от TMR1,TMR2
	bsf	INTCON,T0IE	; разрешаем прерывание от TMR0
	bsf	INTCON,GIE	; глобальное разрешение прерывания
	return

clear_ram
	;clear bank0/2
	movlw	0x20
	movwf	FSR
_clear_loop0
	clrf	INDF
	incf	FSR,F
	btfss	FSR,7
	goto	_clear_loop0

	;clear bank1/3
	movlw	0xA0
	movwf	FSR
_clear_loop1
	clrf	INDF
	incf	FSR,F
	btfsc	FSR,7
	goto	_clear_loop1
	return
;=================================================================================================================
;  ЧТЕНИЕ ИЗ EEPROM ДАННЫХ
;=================================================================================================================

; чтение из EEPROM (начальный адрес в EEADR, конечный адрес в W) в RAM (начальный адрес в FSR)
READ_BYTES
	movwf	EEADR_LAST
	incf	EEADR_LAST

_read_bytes
	BANK3
	bcf	EECON1,EEPGD
	bsf	EECON1,RD	;Читаем
	BANK2
	movf	EEDATA,W		;Из EEDATA в W
	incf	EEADR,f
	movwf	INDF			; записываем регистр
	incf	FSR,f
	movf	EEADR_LAST,W	; адрес последней ячейки + 1
	xorwf	FSR,w
	JZN	_read_bytes

	return

READ
	bcf	STATUS,IRP

	BANK2

;	чтение EEPROM

;	адрес eeprom 	0x00
;	регистры	0x20 - 0x39
	clrf	EEADR	; адрес в области EEPROM
	movlw	h'20'	; начальный адрес регистра в оперативной памяти
	movwf	FSR
	movlw	h'39'	; конечный адрес регистра в оперативной памяти
	call	READ_BYTES

;	адрес eeprom 	0x1A
;	регистры	0xA6 - 0xAB
	movlw	0xA6
	movwf	FSR				;Выбираем адрес регистра в оперативной памяти
	movlw	0xAB
	call	READ_BYTES

;	адрес eeprom 	0x48
;	регистры	0xB0 - 0xD7
	movlw	0x48
	movwf	EEADR
	movlw	0xB0
	movwf	FSR				;Выбираем адрес регистра в оперативной памяти
	movlw	0xD7
	call	READ_BYTES
	
;------------------ расчет временных констант---------------------------
	BANK1
;	проверка на диапазон MODE_LCD1 (0..MAX_MODE_LCD1)
	movf	MODE_LCD1,w
	;addlw	-.0 			; Сперва нормируем значение
	addlw	-(MAX_MODE_LCD1 - .0 + .1) ; Теперь сравниваем с диапазоном
	btfss	STATUS,C
	goto	_mode_lcd1_compare1
	clrf	MODE_LCD1
_mode_lcd1_compare1:

;	проверка на диапазон MODE_LCD2 (0..MAX_MODE_LCD2)
	movf	MODE_LCD2,w
	;addlw	-.0 			; Сперва нормируем значение
	addlw	-(MAX_MODE_LCD2 - .0 + .1) ; Теперь сравниваем с диапазоном
	btfss	STATUS,C
	goto	_mode_lcd2_compare1
	clrf	MODE_LCD2
_mode_lcd2_compare1:

        BANK0
	movlw	.8
	movwf	PCLATH	; страница памяти 1

	movf	ODO_CON1L,w
	movwf	R4
	movf	ODO_CON1H,w
	movwf	R5
	movlw	.2
	movwf	R3
	call	MUL16x8 	; R0:R1:R2 = R5:R4 * R3

	clrf	R5
	movlw	.13
	btfsc	ZUMER_ON,5
	movlw	.26			; для парного впрыска
	movwf	R4
	call	div24_16	; R0:R1:R2 = R0:R1:R2 / R5:R4

	movf	R1,w
	movwf	ODO_CON4H
	movf	R2,w
	movwf	ODO_CON4L

_compare_dates_counter_c:	
	callp	TIME_R	    ; чтение текущего времени
	pageselw $

; todo очистка счетчика текущей поездки по прошествии n минут после завершения предыдущей поездки

;	BANK1
;	movlw	0x21
;	movwf	YEAR_C
;	movlw	0x09
;	movwf	MONTH_C
;	movlw	0x21
;	movwf	DATE_C
;	movlw	0x10
;	movwf	HOURS_C
;	movlw	0x55
;	movwf	MINUTES_C
;	BANK0
;	
;	movlw	0x21
;	movwf	YEAR
;	movlw	0x09
;	movwf	MONTH
;	movlw	0x21
;	movwf	DATE
;	movlw	0x12
;	movwf	HOURS
;	movlw	0x55
;	movwf	MINUTES

		    
; compare two BCD dates (with difference no more, than DIFF_HOURS)

PAUSE_MINUTES	equ	.120 + .1    ; max pause within one trip
DIFF_HOURS	equ	.6	     ; max hours difference (0 ... 24)

	movlw	YEAR_C
	movwf	FSR

; years_diff = year - year_c
; if (years_diff < 0 || years_diff > 1) break;
	movf	YEAR,w
	movwf	R15
	movf	INDF,w
	call	bcd_subtract ; W = YEAR - YEAR_C
	btfss	STATUS,C
	goto	_clearc	    ; if (YEAR < YEAR_C) goto _clearc
	movwf	R14	    ; R14 = YEAR - YEAR_C
	sublw	.01	    
	btfss	STATUS,C    ; if (YEAR_C - YEAR) > 1 goto _clearc
	goto	_clearc

; yday = ydayArray[month] + date; yday_c = ydayArray[month_c] + date_c; yday_diff = 365 * years_diff + yday - ydayc;
; if (yday_diff < 0 || yday_diff > 1) break;
	pageselw _get_yday1

	movf	MONTH,w
	movwf	R15
	call	get_yday    ;R0:R1 - yday (month)
			    
	movf	DATE,w
	movwf	R15
	call	BCD_to_bin
	addwf	R1,f
	btfsc	STATUS,C
	incf	R0,f	    ;R0:R1 - yday

	movf	R0,w	    ;R2:R3 - yday    
	movwf	R2
	movf	R1,w		    
	movwf	R3
	
	decf	FSR,f
	movf	INDF,w
	movwf	R15
	call	get_yday    ;R0:R1 - ydayc (month)
			    
	decf	FSR,f
	movf	INDF,w
	movwf	R15
	call	BCD_to_bin
	addwf	R1,f
	btfsc	STATUS,C
	incf	R0,f	    ;R0:R1 - ydayc
	
	movf	R14,f
	btfsc	STATUS,Z
	goto	_compare_days0
	; yday = 365 + yday
	movlw	low .365
	addwf	R3,f
	btfsc	STATUS,C
	incf	R2,f
	;movlw	high .365
	;addwf	R2,f
	incf	R2,f	
	
_compare_days0
	; yday = yday - ydayc
	movf    R1,W
        subwf   R3
        movf    R0,W
        btfss   STATUS,C
        incfsz  R0,W
        subwf   R2           ; dest = dest - source, WITH VALID CARRY
                             ; (although the Z flag is not valid).

	; if R2:R3 == 0 || R2:R3 == 1 continue comparing
	movf	R2,f
	btfss	STATUS,Z
	goto	_clearc
	movf	R3,w
	sublw	.01
	btfss	STATUS,C
	goto	_clearc

	movf	R3,f
	btfsc	STATUS,Z
	goto	_compare_hours0
	movlw	.24
	movwf	R3	    ; if (days == 0) R3 = 0 else R3 = 24
	
_compare_hours0
; hours_diff = (yday_diff == 1 ? 24 : 0) + hour - hour_c;
; if (hours_diff < 0 || hours_diff > DIFF_HOURS) break;	
	movf	HOURS,w
	movwf	R15
	call	BCD_to_bin
	addwf	R3,f	     ; R3 = (days == 0 ? 0 : 24) + HOURS
	decf	FSR,f
	movf	INDF,w
	movwf	R15
	call	BCD_to_bin   ; W = HOURS_C
	subwf	R3,w	     ; R3 = (days == 0 ? 0 : 24) + HOURS - HOURS_C
	movwf	R3
	sublw	DIFF_HOURS	    
	btfss	STATUS,C     ; if W > 6 goto _clearc
	goto	_clearc

	clrf	R5
	movlw	.60
	movwf	R4
	callp	MUL16x8	     ; R0:R1:R2 = 60 * R3
	pageselw $

_compare_mins0	
; minutes_diff = 60 * hours_diff + minutes - minutes_c
; if (minutes_diff > PAUSE_MINUTES) break;	
	movf	MINUTES,w
	movwf	R15
	call	BCD_to_bin  ; W = MINUTES
	addwf	R2,f
	btfsc	STATUS,C
	incf	R1,f
	
	decf	FSR,f
	movf	INDF,w
	movwf	R15
	call	BCD_to_bin   ; W = MINUTES_C
	subwf	R2,f
	btfss	STATUS,C
	decf	R1
	
	;R1:R2 - minutes diff, compare with PAUSE_MINUTES
	movlw	high PAUSE_MINUTES	;  Compare the High Byte First
	subwf	R1,w
	btfss	STATUS,Z		;  If Result of High Byte Compare
	goto	_end_compare		;  is Equal to Zero, Then Check
	movlw	low PAUSE_MINUTES	;  the Second
	subwf	R2, w
_end_compare
	btfss	STATUS,C		;  Carry Flag Set if PAUSE_MINUTES <= R1:R2
	goto	_init
	
_clearc	
	callp	_CLEAR_C
;------------------ проверка первого запуска БК---------------------------

_init:
	clrf	R7		; регистр для передачи параметра

	; если установлен флаг сигнала ТО счетчиков
	; проверим их значение и если выше предела
	; покажем и озвучим
	clrf	PCLATH		; страница памяти 0
	btfss	ZUMER_ON,2
	return
	btfss	button1 	; если входим в сервис, то пропускаем
	return

	pageselw $
; счетчик моточасов
	BANK1
	movf	mh_limit_L,w
	btfss	STATUS,Z
	goto	_mh_compare_start
	movf	mh_limit_H,w
	btfss	STATUS,Z
	goto	_mh_compare_start
	BANK0
	goto	_mh_compare_end

_mh_compare_start
	movf	mh_limit_L,w
	subwf	mh_L,w

	movf	mh_limit_H,w
	btfss   STATUS,C
	addlw   .1
	subwf   mh_H,w
	BANK0

	btfss   STATUS,C
	goto    _mh_compare_end
	bsf	R7,7
_mh_compare_end

	movlw	.8
	movwf	PCLATH	; страница памяти 1

	movlw	high .1000
	movwf	R5
	movlw	low .1000
	movwf	R4

; счетчик 1
	movf	odo_S1L,w
	movwf	R2
	movf	odo_S1H,w
	movwf	R3
	call	div_16		; R1:R0 = R3:R2 / R5:R4
				; В R3:R2 остаток.
	movlw	TO_1
	movwf	FSR
	movf	INDF,w
	subwf	R0,w
	btfsc	STATUS,C	; если результат вычитания не отрицательный C=1
	bsf	R7,1		; установим флаг

; счетчик 2
	movf	odo_S2L,w
	movwf	R2
	movf	odo_S2H,w
	movwf	R3
	call	div_16		; R1:R0 = R3:R2 / R5:R4
				; В R3:R2 остаток.
	movlw	TO_2
	movwf	FSR
	movf	INDF,w
	subwf	R0,w
	btfsc	STATUS,C	; если результат вычитания не отрицательный C=1
	bsf	R7,2		; установим флаг

; счетчик 3
	movf	odo_S3L,w
	movwf	R2
	movf	odo_S3H,w
	movwf	R3
	call	div_16		; R1:R0 = R3:R2 / R5:R4
				; В R3:R2 остаток.
	movlw	TO_3
	movwf	FSR
	movf	INDF,w
	subwf	R0,w
	btfsc	STATUS,C	; если результат вычитания не отрицательный C=1
	bsf	R7,3		; установим флаг

; счетчик 4
	movf	odo_S4L,w
	movwf	R2
	movf	odo_S4H,w
	movwf	R3
	call	div_16		; R1:R0 = R3:R2 / R5:R4
				; В R3:R2 остаток.
	movlw	TO_4
	movwf	FSR
	movf	INDF,w
	subwf	R0,w
	btfsc	STATUS,C	; если результат вычитания не отрицательный C=1
	bsf	R7,4		; установим флаг

	clrf	PCLATH
	movf	R7,w
	btfss	STATUS,Z
	goto	LCD_ERROR
	return

; get day of year for month
get_yday:
    	call	BCD_to_bin
	movwf	_tmp
	addlw	-.1
	call	_get_yday1
	movwf	R1
	clrf	R0
	movfw	_tmp
	sublw	.9	    ; (9 - _tmp)
	btfss	STATUS,C
	incf	R0,f
	return
	
_get_yday1:	
	addwf	PCL,f	    ; month
_jan	retlw	low .0
_feb	retlw	low .31
_mar	retlw	low .59
_apr	retlw	low .90
_may	retlw	low .120
_jun	retlw	low .151
_jul	retlw	low .181
_aug	retlw	low .212
_sep	retlw	low .243
_oct	retlw	low .273
_nov	retlw	low .304
_dec	retlw	low .334

; convert BCD (R15) to binary (W)
BCD_to_bin:
        swapf   R15, W
        andlw   0x0F            ; W=tens
        movwf   _tmp
        addwf   _tmp, W         ; W=2*tens
        addwf   _tmp, F         ; temp=3*tens (note carry is cleared)
        rlf     _tmp, W         ; W=6*tens
        subwf   R15, W          ; W = 16*tens+ones - 6*tens
	return

;****************************************** 
;bcd_subtract 
; 
; Computes  z = x - y 
; where x,y,z are all 8-bit packed BCD numbers 
; Exits with C=1 (and DC=1 too)  if x>=y 
; and with z=1 if x==y. 
; Note that z can be aliased to x or y so that 
; it's possible to calculate x = x-y or y = x-y 
; 9 cycles (+ return) 
 
;BCD substract R15 = R15 - w
bcd_subtract 
        ;MOVF    y, W     ;W = y 
        SUBWF   R15, W    ;W = x-y 
        RLF     R15, F    ;lsb of z has the carry 
        SKPDC             ;if lsn of x < lsn of y 
        ADDLW  -0x06      ; then convert lsn of the 
                          ; result to BCD. 
        BTFSS   R15, 0    ;Similarly for the msn's 
        ADDLW  -0x60 
        RRF     R15, F    ;Get the carry 
        MOVWF   R15       ;and save the result 

        RETURN 
;==============================================================================================
	; сохранение из RAM (начальный адрес в FSR, конечный адрес в W) в EEPROM (начальный адрес в EEADR)
SAVE_BYTES
	movwf	EEADR_LAST
	incf	EEADR_LAST

_save_bytes_1
	movf	INDF,w
	call 	EESAVE
	call 	SAVE_R

	incf	FSR,f
	BANK2
	movf	EEADR_LAST,W
	xorwf	FSR,w
	JZN	_save_bytes_1

	return

SAVE:
	bcf	INTCON,GIE	; глобальное запрещение прерывания
	bcf	STATUS,IRP
	BANK3
	btfsc	EECON1,WR
	goto	$-1
	BANK0

	btfsc	menu_position,7	; если бит установлен
	goto	SAVE_CONST	; пишем только костанты

	BANK2

;	адрес eeprom	0x00
;	регистры	0x20 - 0x34
	clrf	EEADR
	movlw	0x20
	movwf	FSR
	movlw	0x34
	call	SAVE_BYTES

;	адрес eeprom 	0x48
;	регистры	0xB0 - 0xD7
	movlw	0x48
	movwf	EEADR
	movlw	0xB0
	movwf	FSR
	movlw	0xD7
	call	SAVE_BYTES

	return

;=========================

SAVE_CONST

	BANK2

;	адрес eeprom		0x15
;	регистры		0x35 - 0x39
	movlw   0x15
	movwf	EEADR
	movlw	0x35
	movwf	FSR
	movlw	0x39
	call	SAVE_BYTES

;	адрес eeprom		0x1A
;	регистры		0xA6 - 0xAB
	movlw	0xA6
	movwf	FSR
	movlw	0xAB
	call	SAVE_BYTES

;	адрес eeprom		0x00
;	регистры		0x20 - 0x22
	clrf	EEADR
	movlw	0x20		; основной пробег
	movwf	FSR
	movlw	0x22
	call	SAVE_BYTES

;	адрес eeprom 		0x4F
;	регистры		0xB7 - 0xB8
	movlw	0x4F
	movwf	EEADR
	movlw	0xB7		; лимит моточасов
	movwf	FSR
	movlw	0xB8
	call	SAVE_BYTES

	BANK0

	btfss	menu_position,6	; если бит установлен, запишем адрес датчика
	return

;	адрес eeprom	0x30/0x38/0x40
;	регистры	fROM_ID0 - fROM_ID7

	movlw	h'30'		;адрес начала в EEPROM 1 датчик
	btfsc	fTEMPER_H,0
	movlw	h'38'		;адрес начала в EEPROM 2 датчик
	btfsc	fTEMPER_H,1
	movlw	h'40'		;адрес начала в EEPROM 3 датчик

	BANK2

	bsf	STATUS,IRP
	movwf	EEADR
	movlw	low fROM_ID0
	movwf	FSR		;Выбираем адрес регистра в оперативной памяти
	movlw	low fROM_ID7
	call	SAVE_BYTES

	return

;==============================================================================================
SAVE_R					; ожидание конца записи
	btfsc	EECON1,WR
	goto	$-1
	BANK2
	incf	EEADR,f
	BANK0
	return
;=====================================================================================
EESAVE				;Обязательная процедура записи EEPROM
	BANK2
	movwf	EEDATA
	BANK3
	bcf	EECON1, EEPGD
	bsf	EECON1, WREN	;Разрешаем запись
 	movlw	h'55'			;** Обязательная **
	movwf	EECON2		;**   процедура  **
	movlw	h'aa'			;**      без     **
	movwf	EECON2		;** комментариев **
	bsf	EECON1, WR		;Команда начала записи
	nop
	bcf	EECON1, WREN
	return
;==============================================================================================
SAVE_DATE		; процедура записи даты обнуления счетчика в память
	BANK2
	movwf	EEADR
	bcf	INTCON,GIE	; глобальное запрещение прерывания
	BANK0

	movf	DATE,w
	call 	EESAVE
	call 	SAVE_R

	movf	MONTH,w
	call 	EESAVE
	call 	SAVE_R

	movf	YEAR,w
	call 	EESAVE
	call 	SAVE_R

	BANK0
	bsf	INTCON,GIE	; глобальное разрешение прерывания
	clrf	PCLATH
	return



;=====================================================================================
; сервисный режим изменения констант
;=====================================================================================
; используемые регистры
; fTEMPER_L, fTEMPER_H локальное использование
;
;
srv100 	; ОСНОВНОЕ МЕНЮ
	bsf	menu_position,7	;используем бит при записи костант
	clrf	fTEMP
	; начальный пункт меню
	clrf	fTEMPER_L

	goto	srv103

srv104
	clrf	PCLATH	; страница памяти 0
	call 	SCANP		; выход только по отключению питания
	call	scan
	bsf	PCLATH,4

	btfsc	button_OK1
	goto	srv102
	btfss	button_OK2
	goto	srv104
	gotop	SERVICE_POINT

srv102
	incf	fTEMPER_L,f
	movf	fTEMPER_L,w
	xorlw	0x0C			; макс. уровень меню - 1
	btfss	STATUS,Z
	goto 	srv103
	clrf	fTEMPER_L

srv103
	bcf	PCLATH,4	; страница памяти 0
	call	LCD_CLEAR
	movlw 	0x7F
	movwf	LCD_PLACE

	MSLOVO_XXX	0x41, 0x0B	; СЕРВИС МЕНЮ

	callp	SERVICE_		; показывает текущий пункт меню
	pageselw $

	bcf	button_OK1
	bcf	button_OK2
	goto	srv104
;=====================================================================================
;  	выход через 4 сек. бездействия
; 	интервал считает рег. fTEMP
;	ТОПЛИВНАЯ КОНСТАНТА
srv1
	clrf	PCLATH		; страница памяти 0
	call	scan
	bsf	PCLATH,4	; страница памяти 2
	btfsc	button_OK1
	goto	srv3
	btfss	button_OK2
	goto	srv4
	decf	FUEL_CONST,f
	goto 	$+2

srv3
	incf	FUEL_CONST,f
	clrf	fTEMP		;обнуляем рег. при нажатии на кнопку
	bsf	SEC2_OK

srv4
	btfss	SEC2_OK		; если флаг установлен
	goto	srv1		; обновляем экран через 2 сек

srv2
	clrf	PCLATH		; страница памяти 0
	call	LCD_CLEAR
	bsf	PCLATH,4	; страница памяти 2
	movlw	0x7F
	movwf	LCD_PLACE

	call	srv110

	clrf	PCLATH		; страница памяти 0
	movlw	0xC8
	movwf	LCD_PLACE
	movf	FUEL_CONST,w
	movwf	R3
	clrf	R4
	clrf	R5
	call  B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2
	prv_w	R1
	bsf	ZERO
	call 	ZERO_

	prv_wi	R2
	call 	ZERO_

	prv_w	R2
	call 	LCD
	bsf	PCLATH,4	; страница памяти 2

	bcf	button_OK1
	bcf	button_OK2
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto	srv1
	clrf	fTEMP
	goto	srv103

;=====================================================================================
;	VSS КОНСТАНТА

srv9
	movlw	0x08
	movwf	tTIME

	clrf	PCLATH	; страница памяти 0
;	включаем курсор
	MOVLW	b'00001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11111111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200		;
	bsf	PCLATH,4	; страница памяти 2

	goto	srv5

srv6
	clrf	PCLATH		; страница памяти 0
	call	scan
	bsf	PCLATH,4	; страница памяти 2

	btfsc	button_OK1
	goto	srv6_1

	btfss	button_OK2
	goto	srv8

	incf	tTIME,f
	movf	tTIME,w
	xorlw	0x0D
	btfss	STATUS,Z
	goto 	srv9_1
	movlw	0x08
	movwf	tTIME
	goto 	srv9_1

srv6_1
	callp	VSS_
	pageselw $
	movf	KMH_L,w
	movwf	ODO_CON1L
	movf	KMH_H,w
	movwf	ODO_CON1H

srv9_1
	clrf	fTEMP		;обнуляем рег. при нажатии на кнопку
	bsf	SEC2_OK

srv8
	btfss	SEC2_OK		; если флаг установлен
	goto	srv6		; обновляем экран через 2 сек

srv5
	bcf	PCLATH,4	; страница памяти 0
	call	LCD_CLEAR
	bsf 	PCLATH,4	; страница памяти 2
	movlw	0x7F
	movwf	LCD_PLACE
	call	srv111

	movlw	0xC5
	movwf	LCD_PLACE
	movf	ODO_CON1L,w
	movwf	R3
	movf	ODO_CON1H,w
	movwf	R4
	clrf	R5

	clrf	PCLATH		; страница памяти 0
	call    B2_BCD          ; After conversion the Decimal Number
                               ; in R0,R1,R2
	prv_w	R0
	call 	LCD_INCPLACE
	prv_wi	R1
	call 	LCD_INCPLACE
	prv_w	R1
	call 	LCD_INCPLACE
	prv_wi	R2
	call 	LCD_INCPLACE
	prv_w	R2
	call 	LCD_INCPLACE

	call	KUR			; установка курсора в нужную позицию
	bsf	PCLATH,4	; страница памяти 2

	bcf	button_OK1
	bcf	button_OK2
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto	srv6

	clrf	PCLATH		; страница памяти 0
	clrf	fTEMP
	MOVLW	b'00001111'	; выключаем курсоры
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	bsf	PCLATH,4	; страница памяти 2
	goto	srv103

;=====================================================================================
;	объем бака
srv71
	clrf	PCLATH		; страница памяти 0
	call	scan
	bsf	PCLATH,4	; страница памяти 2

	btfsc	button_OK1
	goto	srv72
	btfss	button_OK2
	goto	srv73
	decf	FUEL_MAX,f
	goto 	$+2

srv72
	incf	FUEL_MAX,f
	clrf	fTEMP		;обнуляем рег. при нажатии на кнопку
	bsf	SEC2_OK
	movlw	.71
	subwf	FUEL_MAX,w
	btfsc	STATUS,C
	clrf	FUEL_MAX

srv73
	btfss	SEC2_OK		; если флаг установлен
	goto	srv71		; обновляем экран через 2 сек

srv70
	clrf	PCLATH		; страница памяти 0
	call	LCD_CLEAR
	bsf	PCLATH,4	; страница памяти 2

	movlw	0x7F
	movwf	LCD_PLACE
	call	srv114

	clrf	PCLATH		; страница памяти 0
	movlw	0xC8
	movwf	LCD_PLACE
	movf	FUEL_MAX,w
	movwf	R3
	clrf	R4
	clrf	R5
	call  	B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2

	prv_wi	R1
	bsf		ZERO
	call 	ZERO_

	prv_w	R1
	call 	ZERO_

	prv_wi	R2
	call 	ZERO_

	prv_w	R2
	call 	LCD

	movlw	л
	btfss	ZUMER_ON,6
	movlw	_l
	call 	LCD_

	bsf	PCLATH,4	; страница памяти 2

	bcf	button_OK1
	bcf	button_OK2
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto	srv71
	clrf	fTEMP

	goto	srv103

;=====================================================================================
;	ТО счетчиков
srv80
; для удобства загрузим счетчик в рег. fTRY
; при выходе сделаем выгрузку обратно
	callp	ODOM_SELECT
	movwf	FSR
	movf	INDF,w
	movwf	fTRY
	pageselw $
	goto	srv85

srv81
	clrf	PCLATH		; страница памяти 0
	call	scan
	bsf	PCLATH,4	; страница памяти 2
	btfsc	button_OK1
	goto	srv82
	btfss	button_OK2
	goto	srv83
	decf	fTRY,f
	goto 	$+2
srv82
	incf	fTRY,f
	clrf	fTEMP		;обнуляем рег. при нажатии на кнопку
	bsf	SEC2_OK
	movlw	.66
	subwf	fTRY,w
	btfsc	STATUS,C
	clrf	fTRY
srv83
	btfss	SEC2_OK		; если флаг установлен
	goto	srv81		; обновляем экран через 2 сек

srv85
	clrf	PCLATH		; страница памяти 0
	call	LCD_CLEAR

	pageselw SERVICE_TO
	movlw	0x7F
	call	SERVICE_TO

	clrf	PCLATH		; страница памяти 0
	movlw	0xC4
	movwf	LCD_PLACE
	movf	fTRY,w
	movwf	R3
	clrf	R4
	clrf	R5
	call  B2_BCD          ; After conversion the Decimal Number
                                ; in R0,R1,R2
	prv_wi	R1
	bsf		ZERO
	call 	ZERO_

	prv_w	R1
	call 	ZERO_

	prv_wi	R2
	call 	ZERO_

	prv_w	R2
	call 	LCD

	MSLOVO_XXX 0xE9,0x05 	; 000км
	pageselw $

	bcf	button_OK1
	bcf	button_OK2
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto	srv81
	clrf	fTEMP

	callp ODOM_SELECT
	movwf	FSR
	movf	fTRY,w
	movwf	INDF
	pageselw $
	goto	srv103
;=====================================================================================
;	Общий пробег
;	рег. tTIME используется нак номер текущего разряда

srv19
	movlw	0x07
	movwf	tTIME

	clrf	PCLATH		; страница памяти 0
;	включаем курсор
	MOVLW	b'00001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11111111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200		;
	bsf	PCLATH,4	; страница памяти 2
	goto	srv14

srv15
	clrf	PCLATH		; страница памяти 0
	call	scan
	bsf	PCLATH,4	; страница памяти 2
	btfsc	button_OK1
	goto	srv16

	btfss	button_OK2
	goto	srv17

	incf	tTIME,f
	movf	tTIME,w
	xorlw	0x0D
	btfss	STATUS,Z
	goto 	srv18
	movlw	0x07
	movwf	tTIME
	goto 	srv18

srv16
	callp	ODOM_
	pageselw $

	movf	KMH_L,w
	movwf	odo_01L
	movf	KMH_H,w
	movwf	odo_01F
	movf	KMH_Htemp,w
	movwf	odo_01H

srv18
	clrf	fTEMP		;обнуляем рег. при нажатии на кнопку
	bsf	SEC2_OK

srv17
	btfss	SEC2_OK		; если флаг установлен
	goto	srv15		; обновляем экран через 2 сек

srv14
	clrf	PCLATH		; страница памяти 0
	call	LCD_CLEAR
	bsf	PCLATH,4	; страница памяти 2

	movlw	0x7F
	movwf	LCD_PLACE
	call	srv112

	clrf	PCLATH		; страница памяти 0

	movlw	0xC4
	movwf	LCD_PLACE

	movf	odo_01L,w
	movwf	R3
	movf	odo_01F,w
	movwf	R4
	movf	odo_01H,w
	movwf	R5
	call    B2_BCD          ; After conversion the Decimal Number
                               ; in R0,R1,R2

	prv_wi	R0
	call 	LCD_INCPLACE
	prv_w	R0
	call 	LCD_INCPLACE
	prv_wi	R1
	call 	LCD_INCPLACE
	prv_w	R1
	call 	LCD_INCPLACE
	prv_wi	R2
	call 	LCD_INCPLACE
	prv_w	R2
	call 	LCD_INCPLACE

	callp	SLOVO20		; км

	clrf	PCLATH		; страница памяти 0
	call	KUR			; установка курсора в нужную позицию
	bsf	PCLATH,4	; страница памяти 2

	bcf	button_OK1
	bcf	button_OK2
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto	srv15

	clrf	PCLATH		; страница памяти 0
	clrf	fTEMP
	MOVLW	b'00001111'	; выключаем курсоры
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	bsf	PCLATH,4	; страница памяти 2

	goto	srv103
;=====================================================================================
;	ФЛАГИ
srv31

	movlw	0x0D
	movwf	tTIME

	clrf	PCLATH		; страница памяти 0
;	включаем курсор
	MOVLW	b'00001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11111111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200		;
	bsf	PCLATH,4	; страница памяти 2
	goto	srv34

srv35
	clrf	PCLATH		; страница памяти 0
	call	scan
	bsf	PCLATH,4	; страница памяти 2
	btfsc	button_OK1
	goto	srv36

	btfss	button_OK2
	goto	srv37

	incf	tTIME,f
	movf	tTIME,w
	xorlw	0x15
	btfss	STATUS,Z
	goto 	srv38
	movlw	0x0D
	movwf	tTIME
	goto 	srv38
;_________________ изменение текущего бита на противоположный
srv36
	movlw	d'8' ; 8 бит.
	movwf fBIT_CNT
	movlw	0x0C
	subwf	tTIME,w
	movwf	_i2c_byte	; помер текущего бита
	bcf	STATUS,C
srv33
	movf	_i2c_byte,w
	xorwf	fBIT_CNT,w
	btfss	STATUS,Z
	goto	srv39

	btfss ZUMER_ON,7	; OK нашли, меняем на противоположный
	goto	$+3
	bcf 	ZUMER_ON,7
	goto	$+2
	bsf 	ZUMER_ON,7
srv39
	rlf	ZUMER_ON,f
	decfsz  fBIT_CNT,f
	goto   srv33
	rlf	ZUMER_ON,f

srv38
	clrf	fTEMP		;обнуляем рег. при нажатии на кнопку
	bsf	SEC2_OK

srv37
	btfss	SEC2_OK		; если флаг установлен
	goto	srv35		; обновляем экран через 2 сек

srv34
	clrf	PCLATH		; страница памяти 0
	call	LCD_CLEAR
	callp	FLAGI_
	pageselw $

	movlw	0xC3
	movwf	LCD_PLACE

	movlw	d'8' ; 8 бит.
	movwf fBIT_CNT
srv30
	movlw	0x31
	btfss	ZUMER_ON,0
	movlw	0x30
	movwf	LCD_LCD

	clrf	PCLATH		; страница памяти 0
	call 	LCD_INCPLACE
	bsf	PCLATH,4	; страница памяти 2

	rrf	ZUMER_ON,f
	decfsz  fBIT_CNT,f
	goto   srv30
	rrf	ZUMER_ON,f

	clrf	PCLATH		; страница памяти 0
	call	KUR	; установка курсора в нужную позицию
	bsf	PCLATH,4	; страница памяти 2

	bcf	button_OK1
	bcf	button_OK2
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto	srv35

	clrf	PCLATH		; страница памяти 0
	clrf	fTEMP
	MOVLW	b'00001111'	; выключаем курсоры
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	bsf	PCLATH,4	; страница памяти 2
	goto	srv103


;=====================================================================================
; сервисный режим регистрации температурных датчиков
;=====================================================================================
srv10
	callp	Get_ROM_ID	; читаем адрес датчика
	clrf	fTEMPER_H	; используем регистр как номер датчика

srv11
	clrf	PCLATH		; страница памяти 0
	call	scan
	bsf	PCLATH,4	; страница памяти 2

	btfss	button_OK2
	goto	srv50
	incf	fTEMPER_H,f	; меняем номер датчика
	movf	fTEMPER_H,w
	xorlw	.3
	btfsc	STATUS,Z
	clrf	fTEMPER_H

	clrf	fTEMP		;обнуляем рег. при нажатии на кнопку
	bsf	SEC2_OK

srv50
	btfss	SEC2_OK		; если флаг установлен
	goto	srv11		; обновляем экран через 2 сек

	clrf	PCLATH		; страница памяти 0
	call	LCD_CLEAR
	bsf	PCLATH,4	; страница памяти 2
	movlw	0x7F
	movwf	LCD_PLACE
	call	srv115
	pageselw $

	movlw 	0xC0
	movwf	LCD_PLACE

	btfsc	CRC_OK
	goto	srv13
	callp	SLOVO_ERROR1 	; ошибка чтения
	pageselw $
	goto	srv52

srv13
	bsf	menu_position,6	;используем бит выхода из процедуры записи
	gotop	TEMP_

srv26	; IN
	movlw	0xD7
	movwf	_tmp
	movlw	0x02
	movwf	_i2c_byte
	goto	srv51

srv27	;OUT
	movlw	0xD9
	movwf	_tmp
	movlw	0x03
	movwf	_i2c_byte
	goto	srv51

srv28	;t'двиг.
	movlw	0x0F
	movwf	_tmp
	movlw	0x07
	movwf	_i2c_byte

srv51
	callp	SLOVO_XXX
	pageselw $
srv52
	bcf	button_OK1
	bcf	button_OK2
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto	srv11
	clrf	fTEMP
	goto	srv103


;=====================================================================================
;	моточасы двигателя

srv90
	movlw	0x09
	movwf	tTIME

;	включаем курсор

	bcf	PCLATH,4

	MOVLW	b'00001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	MOVLW	b'11111111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT
	CALL	tm200		;

	bsf	PCLATH,4

	goto	srv90_1

srv90_2

	bcf	PCLATH,4
	call	scan
	bsf	PCLATH,4

	btfsc	button_OK1
	goto	srv90_3

	btfss	button_OK2
	goto	srv90_5

	incf	tTIME,f
	movf	tTIME,w
	xorlw	0x0D
	btfss	STATUS,Z
	goto 	srv90_4
	movlw	0x09
	movwf	tTIME
	goto 	srv90_4

srv90_3
	callp	 VSS_

	movf	KMH_L,w
	BANK1
	movwf	mh_limit_L
	BANK0
	movf	KMH_H,w
	BANK1
	movwf	mh_limit_H
	BANK0

	pageselw $

srv90_4
	clrf	fTEMP		;обнуляем рег. при нажатии на кнопку
	bsf	SEC2_OK

srv90_5
	btfss	SEC2_OK		; если флаг установлен
	goto	srv90_2		; обновляем экран через 2 сек

srv90_1
	callp	LCD_CLEAR
	movlw	0x80
	movwf	LCD_PLACE

	callp	srv111_

	movlw	0xC6
	movwf	LCD_PLACE
	BANK1
	movf	mh_limit_L,w
	BANK0
	movwf	R3
	BANK1
	movf	mh_limit_H,w
	BANK0
	movwf	R4
	clrf	R5

	callp     B2_BCD          ; After conversion the Decimal Number
                               ; in R0,R1,R2
	prv_wi	R1
	call 	LCD_INCPLACE
	prv_w	R1
	call 	LCD_INCPLACE
	prv_wi	R2
	call 	LCD_INCPLACE
	prv_w	R2
	call 	LCD_INCPLACE

	call	KUR			; установка курсора в нужную позицию

	pageselw $

	bcf	button_OK1
	bcf	button_OK2
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto	srv90_2

	clrf	fTEMP
	MOVLW	b'00001111'	; выключаем курсоры
	MOVWF	LCD_LCD
	callp	LCD_RWT
	MOVLW	b'11001111'
	MOVWF	LCD_LCD
	CALL	LCD_RWT

	gotop		srv103

srv111_
	MSLOVO_YYY	0xC7, 0x0E	; "моточасы двиг."
	return

; константа для вольтметра
srv91
	callp	_ADC_VOLT
	pageselw $

	clrf	fTEMP
	goto	srv91_1

srv91_2

	bcf	PCLATH,4
	call	scan
	bsf	PCLATH,4

	btfsc	button_OK1
	goto	srv91_3

	btfss	button_OK2
	goto	srv91_5

;	button2 pressed
	BANK1
	incf	VCC_CONSTANT,f
	movf	VCC_CONSTANT,w
	xorlw	.230			; макс. 230
	btfsc	STATUS,Z
	decf	VCC_CONSTANT,f
	BANK0

	goto	srv91_4

srv91_3
	;button1 pressed
	BANK1
	decf	VCC_CONSTANT,f
	movf	VCC_CONSTANT,w
	xorlw	.139			; мин. 139
	btfsc	STATUS,Z
	incf	VCC_CONSTANT,f
	BANK0

srv91_4
	clrf	fTEMP		;обнуляем рег. при нажатии на кнопку
	bsf	SEC2_OK

srv91_5
	btfss	SEC2_OK		; если флаг установлен
	goto	srv91_2		; обновляем экран через 2 сек

srv91_1
	callp	LCD_CLEAR
	movlw	0x81
	movwf	LCD_PLACE

	callp	srv91_7

	pageselw LCD_VOLT
	movlw	0xC7
	call	LCD_VOLT

	pageselw $

	bcf	button_OK1
	bcf	button_OK2
	bcf	SEC2_OK
	incf	fTEMP,f
	movlw	.03
	subwf	fTEMP,w
	btfss	STATUS,Z
	goto	srv91_2

	gotop	srv103

srv91_7
	MSLOVO_YYY	0xDB, 0x02	; 12.Uа
	incf	LCD_PLACE,f
	MSLOVO_XXX	0x53, 0x09	; константа
	return


srv40_0
	;бит 7
	callp	SLOVO18	; т.датчик

	incf	LCD_PLACE,f
	movlw	0x07
	movwf	_i2c_byte
	movlw	0x2B
	btfss	ZUMER_ON,7
	movlw	0x24
	movwf	_tmp
	callp	SLOVO_XXX	;DS18b20/DS18s20
	return

srv40	; бит 6
	callp	SLOVO17	; monitor

	incf	LCD_PLACE,f
	movlw	0x03
	movwf	_i2c_byte
	movlw	0x3B
	btfss	ZUMER_ON,6
	movlw	0x3E
	movwf	_tmp
	callp	SLOVO_XXX	;LCD/VLI
	return

srv41	; бит 5
	callp	SLOVO27	; парный впрыск
	pageselw	$

	movlw	0x20
	goto	_bit_on_off

srv42	; бит 4
	callp	SLOVO11	;зум.
	callp	SLOVO12	;тахометр
	pageselw	$

	movlw	0x10
	goto	_bit_on_off

srv43	; бит 3
	callp	SLOVO11	;зум.
	callp	SLOVO13	;скорость
	pageselw	$

	movlw	0x08
	goto	_bit_on_off

srv44 ; бит 2
	callp	SLOVO11	;зум.
	incf	LCD_PLACE,f
	callp	SLOVO16	;ТО

	pageselw	$
	movlw	0x04
	goto	_bit_on_off

srv45	;бит 1
	callp	SLOVO11	;зум.
	incf	LCD_PLACE,f
	callp	SLOVO15	;t двигателя
	pageselw	$

	movlw	0x02
	goto	_bit_on_off

srv46	; бит 0
	callp	SLOVO11	;зум.
	incf	LCD_PLACE,f
	callp	SLOVO14	;кнопок
	pageselw	$

	movlw	0x01
	goto	_bit_on_off

_bit_on_off
	movwf	_tmp
	incf	LCD_PLACE,f
	movlw	0x03
	movwf	_i2c_byte
	movf	ZUMER_ON,w
	andwf	_tmp,f
	btfsc	STATUS,Z
	goto 	_bit_off
	movlw	0x16
	goto	_bit_on_off_1
_bit_off
	movlw	0x19
_bit_on_off_1
	movwf	_tmp
	callp	SLOVO_XXX	;ON/OFF
	return

srv110
	MSLOVO_XXX	0x4C, 0x10	; 1.FUEL константа
	return
srv111
	MSLOVO_XXX	0x5C, 0x05	; 2.VSS
	incf	LCD_PLACE,f
	MSLOVO_XXX	0x53, 0x09	; константа
	return
srv112
	MSLOVO_XXX	0x61, 0x0E	; 3.общий пробег
	return
srv113
	MSLOVO_XXX	0x6F, 0x10	; 4.биты настройки
	return
srv114
	MSLOVO_XXX	0x7F, 0x0E	; 5.емкость бака
	return
srv115
	MSLOVO_XXX	0x8D, 0x10	; 6.инст. датчиков
	return
srv120
	MSLOVO_YYY	0xC5, 0x10	; 7.моточасы двиг
	return
srv116
	MSLOVO_XXX	0x9D, 0x10	; 8.ТО масло двиг
	return
srv117
	MSLOVO_XXX	0xAD, 0x0F	; 9.ТО масло АКПП
	return
srv118
	MSLOVO_XXX	0xBC, 0x10	; 10.ТО возд.фильтр
	return
srv119
	MSLOVO_XXX	0xCC, 0x0B	; 11.ТО свечи
	return
srv121
	MSLOVO_YYY	0xD8, 0x05	; 12.Uа
	incf	LCD_PLACE,f
	MSLOVO_XXX	0x53, 0x09	; константа
	return

;-----------------------------------------------------------------------------

;************************************************************************
;	BCD to Binare conversion
; 	вход: 6-значное BCD число в регистрах R0:R1:R2 (по 4 байта на цифру)
; 	выход:	24 битное бинарное число в регистрах KMH_Htemp:KMH_H:KMH_L
;	регистры R3, R4, R5 для локального использования
;************************************************************************

BCDtoB
	clrf 	KMH_H
	clrf 	KMH_Htemp
	swapf R0,w
	andlw 0F
	movwf	KMH_L
	call	mpy10a

	movf 	R0,w
	call	mpy10b

	swapf	R1,w
	call	mpy10b

	movf	R1,w
	call	mpy10b


	swapf	R2,w
	call	mpy10b

	movf 	R2,w
	andlw	0f
	addwf KMH_L,f
	btfss STATUS,C
	retlw	0
	bcf	STATUS,C
	incf	KMH_H,f
	btfsc STATUS,C
	incf	KMH_Htemp,f
	retlw	0



mpy10b
	andlw	0F
	addwf KMH_L,f
	btfss	STATUS,C
	goto	mpy10a
	bcf	STATUS,C
	incf 	KMH_H,f
	btfsc	STATUS,C
	incf 	KMH_Htemp,f
mpy10a
	bcf	STATUS,C	; умножаем на 2
	rlf 	KMH_L,w
	movwf R5
	rlf	KMH_H,w	; (H_temp,L_temp)=2*N
	movwf R4
	rlf 	KMH_Htemp,w
	movwf R3
	bcf	STATUS,C	; умножаем на 2
	rlf	KMH_L,f
	rlf	KMH_H,f
	rlf	KMH_Htemp,f
	bcf	STATUS,C	; умножаем на 2
	rlf	KMH_L,f
	rlf	KMH_H,f
	rlf	KMH_Htemp,f
	bcf	STATUS,C	; умножаем на 2
	rlf	KMH_L,f
	rlf	KMH_H,f	; (H_byte,L_byte)=8*N
	rlf	KMH_Htemp,f

	movf	R5,w
	addwf KMH_L,f
	btfss	STATUS,C
	goto	mpy10c
	bcf	STATUS,C
	incf 	KMH_H,f
	btfsc	STATUS,C
	incf 	KMH_Htemp,f
	bcf	STATUS,C
mpy10c
	movf	R4,w
	addwf KMH_H,f
	btfsc	STATUS,C
	incf 	KMH_Htemp,f
	movf	R3,w
	addwf KMH_Htemp,f
	retlw	0		; (H_byte,L_byte)=10*N


; обработка полубайта (0-5)
srv29_
	movwf	R6
	andlw	b'00001111'
	movwf	R7
	incf	R7,f
	movlw 0x02
	xorwf	R7,w
	btfsc	STATUS,Z
	clrf	R7
	movlw	b'11110000'
	andwf	R6,w
	iorwf	R7,w
	return

; обработка полубайта
srv29
	movwf	R6
	andlw	b'00001111'
	movwf	R7
	incf	R7,f
	movlw 0x0A
	xorwf	R7,w
	btfsc	STATUS,Z
	clrf	R7
	movlw	b'11110000'
	andwf	R6,w
	iorwf	R7,w
	return

srv20_
	movf	R0,w
	call	srv29_
	movwf	R0
	goto	BCDtoB
srv25
	swapf	R0,w
	call	srv29
	movwf	R0
	swapf	R0,f
	goto	BCDtoB
srv20
	movf	R0,w
	call	srv29
	movwf	R0
	goto	BCDtoB
srv21
	swapf	R1,w
	call	srv29
	movwf	R1
	swapf	R1,f
	goto	BCDtoB
srv22
	movf	R1,w
	call	srv29
	movwf	R1
	goto	BCDtoB
srv23
	swapf	R2,w
	call	srv29
	movwf	R2
	swapf	R2,f
	goto	BCDtoB
srv24
	movf	R2,w
	call	srv29
	movwf	R2
	goto	BCDtoB

; измерение напряжения (мгновенное)
_ADC_VOLT
	call	_adc_volt21
	goto	_adc_volt22

; измерение напряжения (среднее за 0,1 сек)
ADC_VOLT
	btfss	timer01sec
	goto	_adc_volt21
	bcf	timer01sec
_adc_volt22
	BANK1
; расчет voltH,voltL

;	move tVoltH:tVoltF:tVoltL to R0:R1:R2
	movlw	R0
	movwf	FSR
	movf	tVoltH,w
	movwf	INDF
	incf	FSR,f
	movf	tVoltF,w
	movwf	INDF
	incf	FSR,f
	movf	tVoltL,w
	movwf	INDF

;	move tVoltCounterH:tVoltCounterL to R5:R4
	movlw	R4
	movwf	FSR
	movf	tVoltCounterL,w
	movwf	INDF
	incf	FSR,f
	movf	tVoltCounterH,w
	movwf	INDF

	BANK0
; проверка деления на 0
	movf	R4,w
	btfss	STATUS,Z
	goto	_do_div11
	movf	R5,w
	btfsc	STATUS,Z
	goto	_adc_volt21
_do_div11
	callp	div24_16	; R0:R1:R2 = R0:R1:R2 / R5:R4
	BANK1

;	move R1:R2 to voltH:voltL
	movlw	R1
	movwf	FSR
	movf	INDF,w
	movwf	voltH
	incf	FSR,f
	movf	INDF,w
	movwf	voltL

	callp	_adc_volt_min_max
	pageselw $

_adc_volt_inst
	BANK1
	clrf	tVoltCounterL
	clrf	tVoltCounterH

	clrf	tVoltL
	clrf	tVoltF
	clrf	tVoltH

_adc_volt21
	BANK1
	movlw	b'10000000'
	movwf	ADCON1			;все порты А аналоговые
	BANK0

	movlw	b'10001000'
	movwf	ADCON0			; используем вх. А1 ------------------------------------------------------------

	bsf	ADCON0,ADON		; вкл. АЦП
	callp	tm100			; даем зарядиться конденсатору
	pageselw $
	bsf	ADCON0,GO_DONE	; запускаем преобразование
	btfsc 	ADCON0,GO_DONE
	goto 	$-1
	bcf 	ADCON0,ADON		; выкл. АЦП

;24bit+16bit
	BANK1
	movf	ADRESL,w
    addwf	tVoltL,f          ;LS byte
	BANK0
	movf	ADRESH,w
    skpnc
    incfsz	ADRESH,w
	BANK1
    addwf	tVoltF,f          ;
	skpnc
	incf	tVoltH,f

	movlw	.1
	addwf	tVoltCounterL,f
	btfsc	STATUS,C
	incf	tVoltCounterH,f
	movlw	b'00000110'
	movwf	ADCON1			;все порты А цифровые

;test, if voltH:voltL is zero
;	movf	voltH,w
;	btfss	STATUS,Z
;	goto	__skip
;	movf	voltL,w
;	btfss	STATUS,Z
;	goto	__skip
;	movf	tVoltL,w
;	movwf	voltL
;	movf	tVoltF,w
;	movwf	voltH
;	goto	_adc_volt_min_max
;__skip
	BANK0

	return

_adc_volt_min_max
	BANK1
;	update min/max
	movf	voltLmax,w
	subwf	voltL,w
	movf	voltHmax,w
	btfss   STATUS,C
	addlw   .1
	subwf   voltH,w

	btfss   STATUS,C			;skip if volt >= volt_max
	goto    _adc_volt_max_end	;goto if volt <  volt_max
	movf	voltL,w
	movwf	voltLmax
	movf	voltH,w
	movwf	voltHmax
_adc_volt_max_end

	movf	voltLmin,w
	subwf	voltL,w
	movf	voltHmin,w
	btfss   STATUS,C
	addlw   .1
	subwf   voltH,w

	btfsc   STATUS,C			;skip if volt < volt_min
	goto    _adc_volt_min_end	;goto if volt >= volt_min
	movf	voltL,w
	movwf	voltLmin
	movf	voltH,w
	movwf	voltHmin
_adc_volt_min_end

	BANK0

	return


	org	0x1760
#ifdef USE_CUSTOM_CHARS

get_custom_char_data
	movf	_tmp,w
	addwf	PCL,f
	dt	0x05,0x06,0x05,0x00,0x00,0x00,0x01,0x02; kmh[0]
	dt	0x0A,0x15,0x15,0x00,0x10,0x05,0x07,0x01; kmh[1]
	dt	0x07,0x05,0x07,0x00,0x00,0x01,0x02,0x00; omin[0]
	dt	0x00,0x00,0x08,0x10,0x00,0x0A,0x15,0x15; omin[1]
	dt	0x0C,0x14,0x14,0x01,0x02,0x05,0x01,0x01; L100[0]
	dt	0x00,0x00,0x00,0x00,0x00,0x1F,0x15,0x1F; L100[1]
	dt	0x03,0x05,0x05,0x00,0x00,0x01,0x02,0x00; l/h[0]
	dt	0x00,0x00,0x08,0x10,0x00,0x14,0x1C,0x04; l/h[1]
#endif

ODOM_SELECT	; выбор адреса нужного счетчика
	movlw	.07
	subwf	fTEMPER_L,w
	addwf	PCL,f
	retlw	0xA6		; TO 1
	retlw	0xA7		; TO 2
	retlw	0xA8		; TO 3
	retlw	0xA9		; TO 4

TEMP_
	movf	fTEMPER_H,w
	addwf	PCL,f
	goto	srv26		; IN
	goto	srv27		; OUT
	goto	srv28		; t'двиг

FLAGI_
	movlw 	0x7F
	movwf	LCD_PLACE
	movlw	0x0D
	subwf	tTIME,w
	addwf	PCL,f
	goto	srv46
	goto	srv45
	goto	srv44
	goto	srv43
	goto	srv42
	goto	srv41
	goto	srv40
	goto	srv40_0

VSS_
	movf	tTIME,w
	sublw	0x0C
	addwf	PCL,f
	goto	srv24
	goto	srv23
	goto	srv22
	goto	srv21
	goto	srv20_

ODOM_
	movf	tTIME,w
	sublw	0x0C
	addwf	PCL,f
	goto	srv24
	goto	srv23
	goto	srv22
	goto	srv21
	goto	srv20
	goto	srv25


SERVICE_TO
	movlw	0x7F
	movwf	LCD_PLACE

	movlw	.07
	subwf	fTEMPER_L,w
	addwf	PCL,f
	goto	srv116
	goto	srv117
	goto	srv118
	goto	srv119


SERVICE_
	movlw	0xBF
	movwf	LCD_PLACE

	movf	fTEMPER_L,w
	addwf	PCL,f
	goto	srv110
	goto	srv111
	goto	srv112
	goto	srv113
	goto	srv114
	goto	srv115
	goto	srv120
	goto	srv116
	goto	srv117
	goto	srv118
	goto	srv119
	goto	srv121


SERVICE_POINT		; выбор нужного пункта меню
	movf	fTEMPER_L,w
	addwf	PCL,f
	goto	srv2		; FUEL
	goto	srv9		; VSS
	goto	srv19		; пробег
	goto	srv31		; биты
	goto	srv70		; емкость бака
	goto	srv10		; установка датчиков
	goto	srv90		; моточасы двиг
	goto	srv80		; ТО масло двиг.
	goto	srv80		; ТО масло АКПП
	goto	srv80		; ТО возд. фильтр
	goto	srv80		; ТО свечи
	goto	srv91		; U константа



;=====================================================================================
;=====================================================================================
;=====================================================================================
;=================          С Т Р А Н И Ц А  3          =============================
;=====================================================================================
;=====================================================================================
;=====================================================================================

	org 0x1800

MONTH_SELECT
	movlw	0x03
	movwf	_i2c_byte
	; проверка на корректный месяц
	movlw	.19
	subwf	MONTH,w
	btfss	STATUS,C
	goto	$+3
	movlw	.1
	movwf	MONTH

	movf	MONTH,w
	addwf	PCL,f
	nop
	goto	MONTH1
	goto	MONTH2
	goto	MONTH3
	goto	MONTH4
	goto	MONTH5
	goto	MONTH6
	goto	MONTH7
	goto	MONTH8
	goto	MONTH9
	nop
	nop
	nop
	nop
	nop
	nop
	goto	MONTH10
	goto	MONTH11
	goto	MONTH12

DAY_SELECT
	; проверка на корректный день
	movlw	.8
	subwf	DAYOFWEEK,w
	btfss	STATUS,C
	goto	$+3
	movlw	.1
	movwf	DAYOFWEEK

	movf	DAYOFWEEK,w
	addwf	PCL,f
	nop
	goto	day1
	goto	day2
	goto	day3
	goto	day4
	goto	day5
	goto	day6
	goto	day7

MMONTH	macro	start
	movlw 	start
	movwf	_tmp
	call	SLOVO_YYY
	return
	endm

MONTH1
	MMONTH	0x00

MONTH2
	MMONTH	0x03

MONTH3
	MMONTH	0x06

MONTH4
	MMONTH	0x09

MONTH5
	MMONTH	0x0C

MONTH6
	MMONTH	0x0F

MONTH7
	MMONTH	0x12

MONTH8
	MMONTH	0x15

MONTH9
	MMONTH	0x18

MONTH10
	MMONTH	0x1B

MONTH11
	MMONTH	0x1E

MONTH12
	MMONTH	0x21

;-----------------------------------------------------------------------------
MDAY	macro	start,len
	movlw 	start
	movwf	_tmp
	movlw	len
	movwf	_i2c_byte
	call	SLOVO_YYY
	return
	endm

#ifdef LCD_ENGLISH
day1
	MDAY	0x24,0x06
day2
	MDAY	0x2a,0x07
day3
	MDAY	0x31,0x09
day4
	MDAY	0x3a,0x08
day5
	MDAY	0x42,0x06
day6
	MDAY	0x48,0x08
day7
	MDAY	0x50,0x06
#else
day1
	MDAY	0x24,0x0b
day2
	MDAY	0x2f,0x07
day3
	MDAY	0x36,0x05
day4
	MDAY	0x3B,0x07
day5
	MDAY	0x42,0x07
day6
	MDAY	0x49,0x07
day7
	MDAY	0x50,0x0b
#endif
;=====================================================================================
SLOVO1				; сброс?
	movlw 	0x84
	movwf	LCD_PLACE
	MSLOVO_YYY 0x5B,0x06
	return
;=====================================================================================
SLOVO2				; коррекция
	movlw 	0x82
	movwf	LCD_PLACE
	MSLOVO_YYY 0x61,0x0A
	return
;=====================================================================================
SLOVO3_1				; моточасы двиг
	movlw 	0x80
	movwf	LCD_PLACE
	MSLOVO_YYY 0xC7,0x0E
	return

SLOVO3				; масло двигателя /АКПП
	movlw 	0x80
	movwf	LCD_PLACE
#ifdef LCD_ENGLISH
	MSLOVO_XXX 0xAF,0x03
#else
	MSLOVO_XXX 0xAF,0x05
#endif
	pageselw $
	incf	LCD_PLACE,f
	return

SLOVO3_ENGINE			; масло двигателя
#ifdef LCD_ENGLISH
	MSLOVO_XXX 0xA3,0x06
#else
	MSLOVO_YYY 0x6D,0x09
#endif
	return
SLOVO3_AKPP			; масло АКПП
#ifdef LCD_ENGLISH
	MSLOVO_XXX 0xB3,0x04
#else
	MSLOVO_XXX 0xB5,0x04
#endif
	return

;=====================================================================================
SLOVO4				; свечи
	movlw 	0x80
	movwf	LCD_PLACE
	MSLOVO_XXX  0x00, 0x05
	return
;=====================================================================================
SLOVO5				; возд. фильтр
	movlw 	0x80
	movwf	LCD_PLACE
	MSLOVO_XXX  0xBF, 0x0A
	return
;=====================================================================================
SLOVO6	; in
	movwf	LCD_PLACE
	MSLOVO_XXX  0xD7, 0x02
	return
;=====================================================================================
SLOVO7	;out
	movwf	LCD_PLACE
	MSLOVO_XXX  0xD9, 0x03
	return
;=====================================================================================
SLOVO8				; разгон?
	movlw 	0x83
	movwf	LCD_PLACE
	MSLOVO_YYY  0x91, 0x07
	return
;=====================================================================================
SLOVO9				; измерение
	movlw 	0x82
	movwf	LCD_PLACE
	MSLOVO_YYY  0x76, 0x09
	return
;=====================================================================================
SLOVO10	; ОЖ
	movwf	LCD_PLACE
	MSLOVO_YYY  0x7F, 0x02
	return
;=====================================================================================
SLOVO11	; зум.
	MSLOVO_XXX  0x05, 0x04
	return
;=====================================================================================
SLOVO12	; тахометр
#ifdef LCD_ENGLISH
	MSLOVO_YYY  0x81, 0x09
#else
	MSLOVO_YYY  0x81, 0x08
#endif
	return

;=====================================================================================
SLOVO13	; скорость
#ifdef LCD_ENGLISH
	MSLOVO_YYY  0x8b, 0x05
#else
	MSLOVO_YYY  0x89, 0x08
#endif
	return
;=====================================================================================
SLOVO14	; кнопок
	MSLOVO_XXX  0x09, 0x06
	return
;=====================================================================================
SLOVO15	; t'двигателя
	MSLOVO_XXX  0x0F, 0x07
	return
;=====================================================================================
SLOVO16	; ТО
	MSLOVO_XXX  0x32, 0x02
	return
;=====================================================================================
SLOVO17	;monitor
	MSLOVO_XXX  0x34, 0x07
	return
;=====================================================================================
SLOVO18	;т.датчик
	MSLOVO_XXX  0x1C, 0x08
	return
;=====================================================================================
SLOVO19	;км/ч
	MSLOVO_XXX  0xEC, 0x04
	return
;=====================================================================================
SLOVO20	;км
	MSLOVO_XXX  0xEC, 0x02
	return
;=====================================================================================
SLOVO20_1	;м/ч
	MSLOVO_YYY  0xD5, 0x03
	return
;=====================================================================================
SLOVO21	;об/м
	MSLOVO_YYY  0x98, 0x04
	return
;=====================================================================================
SLOVO22	;л/ч
	MSLOVO_YYY  0x9C, 0x03
	return
;=====================================================================================
SLOVO23	;л/100
	MSLOVO_YYY  0x9F, 0x03
	return
;=====================================================================================
SLOVO24	;ожидание старта
	movlw 	0x80
	movwf	LCD_PLACE
	MSLOVO_YYY  0xA2, 0x0f
	return
;=====================================================================================
SLOVO25	;результат
	movlw 	0x82
	movwf	LCD_PLACE
	MSLOVO_YYY  0xB1, 0x09
	return
;=====================================================================================
SLOVO26	;сек.
	MSLOVO_XXX  0xF9, 0x03
	return
;=====================================================================================
SLOVO27	;тип впрыска
	MSLOVO_YYY  0xBA, 0x0B
	return
;=====================================================================================
SLOVO28 ;min
	MSLOVO_YYY  0xDD, 0x01
	return
;=====================================================================================
SLOVO29 ;max
	MSLOVO_YYY  0xDE, 0x01
	return
;=====================================================================================
SLOVO30 ;пробег
	MSLOVO_XXX  0x69, 0x04
	return
SLOVO30_1 ;пр
	MSLOVO_XXX  0x69, 0x02
	return
;=====================================================================================
SLOVO_ERROR1: 	; ошибка чтения
	MSLOVO_XXX  0xDC, 0x0D	; ошибка чтения
	return

;=====================================================================================
SLOVO_ERROR2: 	; ВНИМАНИЕ
	movlw	0x82
	movwf	LCD_PLACE
	MSLOVO_XXX  0xF0, 0x09
	return

;=====================================================================================
SLOVO_ERROR3:	;t'двигателя
	movlw	0xBF
	movwf	LCD_PLACE
	MSLOVO_YYY  0x6B, 0x0B
	return


;=================
	org 0x1BB0
					; используем
					;_tmp сдвиг в таблице,
					;_i2c_byte кол. символов
XXX	call	table_1
	clrf	PCLATH		; страница памяти 0
	call 	LCD_
	incf	_tmp,f
	decf	_i2c_byte,f
	btfsc	STATUS,Z
	return
SLOVO_XXX
	movlw	.31
	btfss	ZUMER_ON,6
	movlw	.29
	movwf	PCLATH		; страница памяти 3
	goto	XXX



;=================
					; используем
					;_tmp сдвиг в таблице,
					;_i2c_byte кол. символов
YYY	call	table_2
	clrf	PCLATH		; страница памяти 0
	call 	LCD_
	incf	_tmp,f
	decf	_i2c_byte,f
	btfsc	STATUS,Z
	return
SLOVO_YYY
	movlw	.30
	btfss	ZUMER_ON,6
	movlw	.28
	movwf	PCLATH		; страница памяти 3
	goto	YYY

;=================
	org 0x1C00
	; копия таблицы table_2 для ВЛИ
	nop
	nop
	dt	Я_,Н_,В_,Ф_,Е_,В_,М_,А_,Р_,А_,П_,Р_,М_,А_,Й_,И_,Ю_,Н_ ;0-11
	dt	И_,Ю_,Л_,А_,В_,Г_,С_,Е_,Н_,О_,К_,Т_,Н_,О_,Я_,Д_,Е_,К_ ;12-23
	dt	П_,О_,Н_,Е_,Д_,Е_,Л_,Ь_,Н_,И_,К_ ;24-2E
	dt	В_,Т_,О_,Р_,Н_,И_,К_ ;2F-35
	dt	С_,Р_,Е_,Д_,А_ ;36-3A
	dt	Ч_,Е_,Т_,В_,Е_,Р_,Г_ ;3B-41
	dt	П_,Я_,Т_,Н_,И_,Ц_,А_ ;42-48
	dt	С_,У_,Б_,Б_,О_,Т_,А_ ;49-4F
	dt	В_,О_,С_,К_,Р_,Е_,С_,Е_,Н_,Ь_,Е_ ;50-5A
	dt	С_,Б_,Р_,О_,С_,_QUEST ;5B-60
	dt	К_,О_,Р_,Р_,Е_,К_,Ц_,И_,Я_,_QUEST ;61-6A
	dt	_t,0x27,Д_,В_,И_,Г_,А_,Т_,Е_,Л_,Я_ ;6B-75
	dt	И_,З_,М_,Е_,Р_,Е_,Н_,И_,Е_ ;76-7E
	dt	О_,Ж_ ;7F-80
	dt	Т_,А_,Х_,О_,М_,Е_,Т_,Р_ ;81-88
	dt	С_,К_,О_,Р_,О_,С_,Т_,Ь_ ;89-90
	dt	Р_,А_,З_,Г_,О_,Н_,_QUEST ;91-97
	dt	_p,_r,_SLASH,_m,_l,_SLASH,_h,h'B9',h'B0',h'B0' ;98-A1
	dt	О_,Ж_,И_,Д_,А_,Н_,И_,Е_,_SPACE,С_,Т_,А_,Р_,Т_,А_ ;A2-B0
	dt	Р_,Е_,З_,У_,Л_,Ь_,Т_,А_,Т_ ;B1-B9
	dt	П_,А_,Р_,Н_,Ы_,Й_,_SPACE,В_,П_,Р_,_POINT ;BА-С4
	dt	0x37,_POINT,М_,О_,Т_,О_,Ч_,А_,С_,Ы_,_SPACE,Д_,В_,И_,Г_,_POINT ;C5-D4
	dt	М_,_SLASH,Ч_; D5-D7
	dt	0x31,0x32,_POINT,_U,_a ; D8-DC
	dt	_GREATER; DD
	dt	_LOWER; DE


;=================
	org 0x1D00
	; копия таблицы table_1 для ВЛИ
	nop
	nop
	dt	С_,В_,Е_,Ч_,И_ ;0-4
	dt 	З_,У_,М_,_POINT,К_,Н_,О_,П_,О_,К_ ;5-0E
	dt	_t,0x27,Д_,В_,И_,Г_,_POINT ;0F-15
	dt	_SPACE,_O,_N,_O,_F,_F ;16-2B
	dt	Т_,_POINT,Д_,А_,Т_,Ч_,И_,К_ ;1C-23
	dt	_D,_S,0x31,0x38,_b,0x32,0x30 ;24-2A
	dt	_D,_S,0x31,0x38,_s,0x32,0x30 ;2B-31
	dt	Т_,О_ ;32-33
	dt	_m,_o,_n,_i,_t,_o,_r,_L,_C,_D,_V,_L,_I ;34-40
	dt	С_,Е_,Р_,В_,И_,С_,_SPACE,М_,Е_,Н_,Ю_ ;41-4B
	dt	0x31,_POINT,_F,_U,_E,_L,_SPACE,К_,О_,Н_,С_,Т_,А_,Н_,Т_,А_ ;4C-5B
	dt	0x32,_POINT,_V,_S,_S	;5С-60
	dt	0x33,_POINT,О_,Б_,Щ_,И_,Й_,_SPACE,П_,Р_,О_,Б_,Е_,Г_	;61-6E
	dt	0x34,_POINT,Б_,И_,Т_,Ы_,_SPACE,Н_,А_,С_,Т_,Р_,О_,Й_,К_,И_	;6F-7E
	dt	0x35,_POINT,Е_,М_,К_,О_,С_,Т_,Ь_,_SPACE,Б_,А_,К_,А_	;7F-8C
	dt	0x36,_POINT,И_,Н_,С_,Т_,_POINT,_SPACE,Д_,А_,Т_,Ч_,И_,К_,О_,В_	;8D-9C
	dt	0x38,_POINT,М_,А_,С_,Л_,О_,_SPACE,Д_,В_,И_,Г_,_POINT,_SPACE,Т_,О_	;9D-AC
	dt	0x39,_POINT,М_,А_,С_,Л_,О_,_SPACE,А_,К_,П_,П_,_SPACE,Т_,О_	;AD-BB
	dt	0x31,0x30,_POINT,В_,О_,З_,_POINT,Ф_,И_,Л_,Ь_,Т_,Р_,_SPACE,Т_,О_	;BC-CB
	dt	0x31,0x31,_POINT,С_,В_,Е_,Ч_,И_,_SPACE,Т_,О_	;CC-D6
	dt	_I,_N,_O,_U,_T ;D7-DB
	dt	О_,Ш_,И_,Б_,К_,А_,_SPACE,Ч_,Т_,Е_,Н_,И_,Я_	;DC-E8
	dt	0x30,0x30,0x30,_k,_m,_SLASH,_h	;E9-EF
	dt	В_,Н_,И_,М_,А_,Н_,И_,Е_,0x21,С_,Е_,К_	;F0-FB


;=================
#ifdef LCD_ENGLISH
	org 0x1E00
table_2
	movf	_tmp,w
	addwf	PCL,f
	dt	_j,_a,_n,_f,_e,_b,_m,_a,_r,_a,_p,_r,_m,_a,_y,_j,_u,_n ;0-11
	dt	_j,_u,_l,_a,_u,_g,_s,_e,_p,_o,_c,_t,_n,_o,_v,_d,_e,_c ;12-23
	dt	_m,_o,_n,_d,_a,_y ;24-29
	dt	_t,_u,_e,_s,_d,_a,_y ;2A-30
	dt	_w,_e,_d,_n,_e,_s,_d,_a,_y ;31-39
	dt	_t,_h,_u,_r,_s,_d,_a,_y ;3A-41
	dt	_f,_r,_i,_d,_a,_y ;42-47
	dt	_s,_a,_t,_u,_r,_d,_a,_y ;48-4F
	dt	_s,_u,_n,_d,_a,_y,_SPACE,_SPACE,_SPACE,_SPACE,_SPACE ;50-55
	dt	_r,_e,_s,_e,_t,_QUEST ;5B-60
	dt	_c,_o,_r,_r,_e,_c,_t,_i,_n,_QUEST ;61-6A
	dt	_t,0x27,_e,_n,_g,_i,_n,_e,_SPACE,_SPACE,_SPACE ;6B-75
	dt	_m,_e,_a,_s,_u,_r,_i,_n,_g ;76-7E
	dt	_C,_O ;7F-80
	dt	_t,_a,_c,_h,_o,_m,_e,_t,_r,_SPACE ;81-8a
	dt	_s,_p,_e,_e,_d,_SPACE ;8b-90
	dt	_a,_c,_c,_e,_l,_QUEST,_SPACE ;91-97
	dt	_r,_e,_v,0xE9,_l,_SLASH,_h,_l,_SLASH,0xA1 ;98-A1
	dt	_SPACE,_w,_a,_i,_t,_i,_n,_g,_SPACE,_s,_t,_a,_r,_t,_SPACE ;A2-B0
	dt	_SPACE,_SPACE,_r,_e,_s,_u,_l,_t,_SPACE ;B1-B9
	dt	_d,_u,_a,_l,_SPACE,_i,_n,_j,_POINT,_SPACE,_SPACE ;BА-С4
	dt	0x37,_POINT,_e,_n,_g,_i,_n,_e,_SPACE,_h,_o,_u,_r,_s,_SPACE,_SPACE ;C5-D4
	dt	_m,_SLASH,_h; D5-D7
	dt	0x31,0x32,_POINT,_U,а ; D8-DC
	dt	_GREATER; DD
	dt	_LOWER; DE

;=================
	org 0x1F00
table_1
	movf	_tmp,w
	addwf	PCL,f
	dt	_s,_p,_a,_r,_k ;0-4
	dt 	_s,_n,_d,_SPACE,_k,_e,_y,_s,_SPACE,_SPACE ;5-0E
	dt	_t,0x27,_e,_n,_g,_n,_POINT ;0F-15
	dt	_SPACE,_O,_N,_O,_F,_F ;16-2B
	dt	_t,_POINT,_s,_e,_n,_s,_o,_r ;1C-23
	dt	_D,_S,0x31,0x38,_b,0x32,0x30 ;24-2A
	dt	_D,_S,0x31,0x38,_s,0x32,0x30 ;2B-31
	dt	_T,_O ;32-33
	dt	_m,_o,_n,_i,_t,_o,_r,_L,_C,_D,_V,_L,_I ;34-40
	dt	_S,_E,_R,_V,_I,_C,_E,_SPACE,_SPACE,_SPACE,_SPACE ;41-4B
	dt	0x31,_POINT,_F,_U,_E,_L,_SPACE,_c,_o,_n,_s,_t,_a,_n,_t,_SPACE ;4C-5B
	dt	0x32,_POINT,_V,_S,_S	;5С-60
	dt	0x33,_POINT,_t,_o,_t,_a,_l,_SPACE,_t,_r,_i,_p,_SPACE,_SPACE	;61-6E
	dt	0x34,_POINT,_s,_e,_t,_t,_i,_n,_g,_s,_SPACE,_b,_i,_t,_s,_SPACE	;6F-7E
	dt	0x35,_POINT,_f,_u,_e,_l,_SPACE,_c,_a,_p,_a,_c,_i,_t	;7F-8C
	dt	0x36,_POINT,_s,_e,_n,_s,_o,_r,_s,_SPACE,_i,_n,_s,_t,_a,_l	;8D-9C
	dt	0x38,_POINT,_o,_i,_l,_SPACE,_e,_n,_g,_i,_n,_e,_SPACE,_T,_O,_SPACE	;9D-AC
	dt	0x39,_POINT,_o,_i,_l,_SPACE,_g,_e,_a,_r,_SPACE,_T,_O,_SPACE,_SPACE	;AD-BB
	dt	0x31,0x30,_POINT,_a,_i,_r,_SPACE,_f,_i,_l,_t,_e,_r,_SPACE,_T,_O	;BC-CB
	dt	0x31,0x31,_POINT,_s,_p,_a,_r,_k,_SPACE,Т,О	;CC-D6
	dt	_I,_N,_O,_U,_T ;D7-DB
	dt	_e,_r,_r,_o,_r,_SPACE,_r,_e,_a,_d,_i,_n,_g	;DC-E8
	dt	0x30,0x30,0x30,_k,_m,_SLASH,_h	;E9-EF
	dt	_A,_T,_T,_E,_N,_T,_I,_O,_N,_s,_e,_c	;F0-FB
#else
	org 0x1E00
table_2
	movf	_tmp,w
	addwf	PCL,f
	dt	zz,н,в,ф,е,в,м,а,р,а,п,р,м,а,й,и,ю,н ;0-11
	dt	и,ю,л,а,в,г,с,е,н,о,к,т,н,о,zz,д,е,к ;12-23
	dt	п,о,н,е,д,е,л,ь,н,и,к ;24-2E
	dt	в,т,о,р,н,и,к ;2F-35
	dt	с,р,е,д,а ;36-3A
	dt	ч,е,т,в,е,р,г ;3B-41
	dt	п,zz,т,н,и,ц,а ;42-48
	dt	с,у,б,б,о,т,а ;49-4F
	dt	в,о,с,к,р,е,с,е,н,ь,е ;50-5A
	dt	с,б,р,о,с,_QUEST ;5B-60
	dt	к,о,р,р,е,к,ц,и,zz,_QUEST ;61-6A
	dt	_t,0x27,д,в,и,г,а,т,е,л,zz ;6B-75
	dt	и,з,м,е,р,е,н,и,е ;76-7E
	dt	О,Ж ;7F-80
	dt	т,а,х,о,м,е,т,р ;81-88
	dt	с,к,о,р,о,с,т,ь ;89-90
	dt	р,а,з,г,о,н,_QUEST ;91-97
	dt	о,б,_SLASH,м,л,_SLASH,ч,л,_SLASH,h'7B' ;98-A1
	dt	о,ж,и,д,а,н,и,е,_SPACE,с,т,а,р,т,а ;A2-B0
	dt	р,е,з,у,л,ь,т,а,т ;B1-B9
	dt	п,а,р,н,ы,й,_SPACE,в,п,р,_POINT ;BА-С4
	dt	0x37,_POINT,м,о,т,о,ч,а,с,ы,_SPACE,д,в,и,г,_POINT ;C5-D4
	dt	м,_SLASH,ч; D5-D7
	dt	0x31,0x32,_POINT,_U,а ; D8-DC
	dt	_ARROW_DOWN; DD
	dt	_ARROW_UP; DE

;=================
	org 0x1F00
table_1
	movf	_tmp,w
	addwf	PCL,f
	dt	с,в,е,ч,и ;0-4
	dt 	з,у,м,_POINT,к,н,о,п,о,к ;5-0E
	dt	_t,0x27,д,в,и,г,_POINT ;0F-15
	dt	_SPACE,_O,_N,_O,_F,_F ;16-2B
	dt	т,_POINT,д,а,т,ч,и,к ;1C-23
	dt	_D,_S,0x31,0x38,_b,0x32,0x30 ;24-2A
	dt	_D,_S,0x31,0x38,_s,0x32,0x30 ;2B-31
	dt	Т,О ;32-33
	dt	_m,_o,_n,_i,_t,_o,_r,_L,_C,_D,_V,_L,_I ;34-40
	dt	С,Е,Р,В,И,С,_SPACE,М,Е,Н,Ю ;41-4B
	dt	0x31,_POINT,_F,_U,_E,_L,_SPACE,к,о,н,с,т,а,н,т,а ;4C-5B
	dt	0x32,_POINT,_V,_S,_S	;5С-60
	dt	0x33,_POINT,о,б,щ,и,й,_SPACE,п,р,о,б,е,г	;61-6E
	dt	0x34,_POINT,б,и,т,ы,_SPACE,н,а,с,т,р,о,й,к,и	;6F-7E
	dt	0x35,_POINT,е,м,к,о,с,т,ь,_SPACE,б,а,к,а	;7F-8C
	dt	0x36,_POINT,и,н,с,т,_POINT,_SPACE,д,а,т,ч,и,к,о,в	;8D-9C
	dt	0x38,_POINT,м,а,с,л,о,_SPACE,д,в,и,г,_POINT,_SPACE,Т,О	;9D-AC
	dt	0x39,_POINT,м,а,с,л,о,_SPACE,А,К,П,П,_SPACE,Т,О	;AD-BB
	dt	0x31,0x30,_POINT,в,о,з,_POINT,ф,и,л,ь,т,р,_SPACE,Т,О	;BC-CB
	dt	0x31,0x31,_POINT,с,в,е,ч,и,_SPACE,Т,О	;CC-D6
	dt	_I,_N,_O,_U,_T ;D7-DB
	dt	о,ш,и,б,к,а,_SPACE,ч,т,е,н,и,zz	;DC-E8
	dt	0x30,0x30,0x30,к,м,_SLASH,ч	;E9-EF
	dt	В,Н,И,М,А,Н,И,Е,0x21,с,е,к	;F0-FB
#endif
;------------------------------------------------------------------------------------------------------------

de24	macro num
	de  ((num) >> .16) & 0xFF
	de  ((num) >>  .8) & 0xFF
	de  ((num) >>  .0) & 0xFF
	endm

de16	macro num
	de  ((num) >>  .8) & 0xFF
	de  ((num) >>  .0) & 0xFF
	endm

; real data eeprom
#define REAL_EEPROM
#ifdef REAL_EEPROM
	org 	0x2100
	de	0x04,0x9F,0x7F,0x00,0x0A,0x0A,0x16,0x13		; 0x00
	de	0x99,0x13,0x99,0x0A,0xAA,0x00,0x64,0x1B
	de	0x90,0x28,0x1A,0x13,0x16,0x6E,0x3E,0x80		; 0x10
	de	0x61,0x3F,0x0A,0x28,0x0A,0x0A,0xA6,0x00
	de	0x22,0x01,0x21,0xFF,0xFF,0xFF,0x21,0x03		; 0x20
	de	0x20,0x07,0x01,0x20,0xFF,0xFF,0xFF,0xFF
	de	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF		; 0x30
	de	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
	de	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF		; 0x40
	de	0x00,0x02,0x02,0x02,0x3A,0x00,0x70,0x00
	de	0x00,0x01,0x22,0x3A,0xDD,0x0F,0x11,0x64		; 0x50
	de	0x3D,0x00,0x4F,0xC8,0x00,0x00,0x00,0x00
	de	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00		; 0x60
	de	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00		; 
#else
#ifndef PROTEUS_SIM
	org 	0x2100
	de24	TOTAL_KM	; общий пробег
	de16	.103		; суточный пробег
	de16	.0		; сервисные счетчики
	de16	.0
	de16	.0
	de16	.0

	de16	.1360		; FUEL_00H / FUEL_00L
	de16	.12800		; предварительный счетчик суточного пробега
	de16	.0		; FUEL_TMP2 / FUEL_TMP1

	de16	.0		; предварительный счетчик основного пробега

	de	FUEL_CONSTANT	; FUEL_CONST топливная константа

	de16	PULSES_PER_KM	; число импульсов на 1 км

	de b'01100000'	; регистр флагов: озвучки событий, типы датчиков и т.д.
				; 7 - тип датчика температуры (0=B,1=S)
				; 6 - тип экрана (0=ВЛИ, 1=ЖКИ)
				; 5 - тип впрыска
				; 4 - ,
				; 3 -
				; 2 - предупреждение и звук по сервис-счетчикам
				; 1 - звук температура ОЖ > 102
				; 0 - звук кнопок
	de	.63		; объем бака
	de	.10		; сервисный счетчик 1
	de	.40		; сервисный счетчик 2
	de	.10		; сервисный счетчик 3
	de	.10		; сервисный счетчик 4
	de	VOLT_CONSTANT
	de	0xFF

	org	0x2148
;	время движения (время работы двигателя), единица измерения - 2 сек (счетчик A)
	de24	.195000
;	предварительный счетчик моточасов
	de16	.0
;	количество моточасов
	de16	.0
;	лимит	моточасов
	de16	.0
;	суточный пробег (B)
	de16	.263
;	предварительный счетчик суточного пробега
	de16	.3500
;	счетчик суточного расхода (B) FUEL_00H_B / FUEL_00L_B
	de16	.3310
;	предварительный счетчик суточного расхода (B) FUEL_TMP2_B / FUEL_TMP1_B
	de16	.0
;	время движения (время работы двигателя), единица измерения - 2 сек (счетчик B)
	de24	.17430

#endif
#endif
; ======================================= КАРТА ПАМЯТИ EEPROM ===============================================
;
; 00 odo_00H основной				10 	odo_00Ltemp суточного пр. 		20 день сервисный сч. 1
; 01 odo_00F пробег				11 	FUEL_TMP2 предварительный		21 месяц сервисный сч. 1
; 02 odo_00L 3 байта 				12	FUEL_TMP1 счетчик топлива		22 год сервисный сч. 1
; 03 odo_00H суточный				13 	odo_01Htemp предварительный счетчик	23 день сервисный сч. 2
; 04 odo_00L пробег				14 	odo_01Ltemp основного пробега		24 месяц сервисный сч. 2
; 05 odo_S1H сервисный				15 	FUEL_CONST топливная константа		25 год сервисный сч. 2
; 06 odo_S1L счетчик 1				16 	ODO_CON1H число				26 день сервисный сч. 3
; 07 odo_S2H сервисный 				17	ODO_CON1L импульсов на 1 км		27 месяц сервисный сч. 3
; 08 odo_S2L счетчик 2				18	ZUMER_ON флаги 				28 год сервисный сч. 3
; 09 odo_S3H сервисный				19	FUEL_MAX объем бака			29 день сервисный сч. 4
; 0A odo_S3L счетчик 3				1A	TO-1	ТО масло двиг.	(0xA6)		2A месяц сервисный сч. 4
; 0В odo_S4H сервисный				1B	TO-2	ТО масло АКПП	(0xA7)		2B год сервисный сч. 4
; 0С odo_S4L счетчик 4				1C	TO-3	ТО свечи	(0xA8)
; 0D FUEL_00H счетчик расхода			1D	TO-4	ТО возд. фильтр	(0xA9)
; 0E FUEL_00L топлива				1E	константа Vcc		(0xAA)
; 0F odo_00Htemp пред. счетчик			1F	резерв			(0xAB)
;
;
; 30-37 серийный номер датчика 1
; 38-3F серийный номер датчика 2
; 40-47 серийный номер датчика 3
;
; 48 time_H 	время (счетчик A)	(0xB0)
; 49 time_F 	движения (2 сек)	(0xB1)
; 4A time_L 	3 байта			(0xB2)
; 4B mh_Htemp	предварительный счетчик	(0xB3)
; 4C mh_Ltemp	часов работы двигателя	(0xB4)
; 4D mh_H	счетчик моточасов	(0xB5)
; 4E mh_L	2 байта			(0xB6)
; 4F mh_limit_H	лимит моточасов		(0xB7)
; 50 mh_limit_L 2 байта			(0xB8)
; 51 odo_00H_B  суточный		(0xB9)
; 52 odo_00L_B  пробег (B)		(0xBA)
; 53 odo_00Htemp_B пред. счетчик	(0xBB)
; 54 odo_00Ltemp_B суточного пр. (B)	(0xBC)
; 55 FUEL_00H_B счетчик расхода		(0xBD)
; 56 FUEL_00L_B топлива	(B)		(0xBE)
; 57 FUEL_TMP2_B предварительный	(0xBF)
; 58 FUEL_TMP1_B счетчик топлива (B)	(0xC0)
; 59 time_H_2 	время (счетчик B)	(0xC1)
; 5A time_F_2 	движения (2 сек)	(0xC2)
; 5B time_L_2 	3 байта			(0xC3)
; 5C MODE_LCD1	режим экрана 1 (хх/дв)	(0xC4)
; 5D MODE_LCD2	режим экрана 2		(0xC5)
; 5E odo_00H_C	параметры пробега текущей поездки (0xC6)
; 5F odo_00L_C				(0xC7)
; 60 odo_00Htemp_C			(0xC8)
; 61 odo_00Ltemp_C			(0xC9)
; 62 time_H_C	время текущей поездки	(0xCA) 
; 63 time_F_C				(0xCB)
; 64 time_L_C				(0xCC)
; 65 FUEL_00H_C	топливо текущей поездки	(0xCD)	
; 66 FUEL_00L_C				(0xCE)
; 67 FUEL_TMP2_C			(0xCF)
; 68 FUEL_TMP1_C			(0xD0)
; 69 MINUTES_C	дата завершения поездки	(0xD1)
; 6A HOURS_C				(0xD2)
; 6B DATE_C				(0xD3)
; 6C MONTH_C				(0xD4)
; 6D YEAR_C				(0xD5)
; .. резерв
; 6F резерв				(0xD7)

	END
