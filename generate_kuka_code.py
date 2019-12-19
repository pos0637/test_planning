# -*- coding: utf-8 -*-

import csv


def write_src_header(file):
    file.write(
        '''DEF  My_Job ( )

;FOLD INI
CONTINUE
IF NOT $ON_PATH THEN
  ;FOLD BASISTECH INI
    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
    INTERRUPT ON 3 
    BAS (#INITMOV,0 )
  ;ENDFOLD (BASISTECH INI)
  ;FOLD USER INI
    ;Make your modifications here

  ;ENDFOLD (USER INI)
ENDIF
;ENDFOLD (INI)

;FOLD SET JOBINFO HTML FILENAME
;  APP_sHTML_JOBINFO[] = "My_Project\My_Job.htm"
;ENDFOLD

;FOLD CHECK DAT-FILE GENERATION NUMBER
  IF APP_GENNUMBER <> 1962437 THEN
    LOOP
      MsgQuit("InconsistenceSRCandDAT",,,,1)
      HALT
    ENDLOOP
  ENDIF
;ENDFOLD


;fold Jobinfo
;Job information: My_Job
;Product : 3.0.0.82
;Date: 2019-10-28-17_50
;Project name: My_Project
;Author: My Author
;Company: My Company
;Division: My Division
;Comment: My Comment
;endfold

;fold Cellmap
; KR3_R540
;endfold
;fold Axes definitions
; Axis1 [-170 170] Rot [deg] (KR3_R540 : A1)
; Axis2 [-170 50] Rot [deg] (KR3_R540 : A2)
; Axis3 [-110 155] Rot [deg] (KR3_R540 : A3)
; Axis4 [-175 175] Rot [deg] (KR3_R540 : A4)
; Axis5 [-120 120] Rot [deg] (KR3_R540 : A5)
; Axis6 [-350 350] Rot [deg] (KR3_R540 : A6)
;endfold
;fold RobRoot, Tools and Bases used
; $ROBROOT = {x 0,y 0,z 0,a 0,b 0,c 0}
;endfold
;fold HomePositions
;endfold
        '''
    )


def write_src_data(file, data):
    for i in range(len(data)):
        file.write(
            """
;FOLD PTP P%s CONT Vel= 100 %% PDATP%s Tool[1] Base[1] ;%%{PE}%%R 8.3.31,%%MKUKATPBASIS,%%CMOVE,%%VPTP,%%P 1:PTP, 2:P%s, 3:C_DIS, 5:100, 7:PDATP%s
$BWDSTART = FALSE
PDAT_ACT=PPDATP%s
FDAT_ACT=FP%s
BAS (#PTP_PARAMS,100)
PTP  XP%s C_DIS
;ENDFOLD
""" % (i, i, i, i, i, i, i)
        )


def write_src_tail(file):
    file.write('END')


def write_dat_header(file):
    file.write(
        '''&ACCESS RVP
&PARAM EDITMASK = *
DEFDAT  My_Job
;FOLD EXTERNAL DECLARATIONS;%%{PE}%%MKUKATPBASIS,%%CEXT,%%VCOMMON,%%P
;FOLD BASISTECH EXT;%%{PE}%MKUKATPBASIS,%CEXT,%VEXT,%P
EXT  BAS (BAS_COMMAND  :IN,REAL  :IN )
DECL INT SUCCESS
;ENDFOLD (BASISTECH EXT)
;FOLD USER EXT;%%{E}%%MKUKATPUSER,%CEXT,%VEXT,%P
;Make your modifications here

;ENDFOLD (USER EXT)
;ENDFOLD (EXTERNAL DECLARATIONS)

INT APP_GENNUMBER = 1962437
INT APP_ANSWER
INT APP_OFFSET
DECL STATE_T APP_STATE
''')


def get_float(value):
    return '%.2f' % value


def write_dat_data(file, data):
    deltaX = 0
    deltaY = 0
    deltaZ = 0

    for i in range(len(data)):
        file.write(
            """
DECL LDAT LCPDATP%s={VEL 1,ACC 100,APO_DIST 100,APO_FAC 50.0,ORI_TYP #VAR,CIRC_TYP #BASE,JERK_FAC 50.0}
DECL FDAT FP%s={TOOL_NO 1,BASE_NO 1,IPO_FRAME #BASE,POINT2[] " ",TQ_STATE FALSE}
DECL E6POS XP%s={X %s,Y %s,Z %s,A %s,B %s,C %s,S %s,T %s,E1 0,E2 0,E3 0,E4 0,E5 0,E6 0}
""" % (i, i, i, get_float(float(data[i][0]) + deltaX), get_float(float(data[i][1]) + deltaY), get_float(float(data[i][2]) + deltaZ), float(data[i][3]), float(data[i][4]), float(data[i][5]), float(data[i][6]), float(data[i][7])))


def write_dat_tail(file):
    file.write('ENDDAT')


def execute():
    reader = csv.reader(open('./output/points.txt'))
    rows = []
    for row in reader:
        rows.append(row)
        print(row)

    file = open('./output/demo.src', 'w')
    write_src_header(file)
    write_src_data(file, rows)
    write_src_tail(file)
    file.close()

    file = open('./output/demo.dat', 'w')
    write_dat_header(file)
    write_dat_data(file, rows)
    write_dat_tail(file)
    file.close()
