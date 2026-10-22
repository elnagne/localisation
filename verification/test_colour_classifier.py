"""Production NXT classifier regression; uses synthetic captures and no hardware."""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
colours={'black':(200,180,190),'white':(750,750,840),'red':(710,373,397),
         'green':(320,428,310),'yellow':(740,720,420),'blue':(300,450,650)}
with tempfile.TemporaryDirectory(prefix='c85_colours_') as name:
    work=Path(name)
    for fixture in ('separated','overlap'):
        folder=work/fixture
        folder.mkdir()
        for colour,rgb in colours.items():
            if fixture=='overlap' and colour=='green': rgb=colours['black']
            text='sensor NXT RGB\nsample R G B ambient\n'
            for index in range(1,31): text+=f'{index} {rgb[0]} {rgb[1]} {rgb[2]} 0\n'
            (folder/(colour+'.txt')).write_text(text)
    executable=work/'classifier'
    subprocess.run(['g++',str(ROOT/'verification/colour_classifier_test.cpp'),
                    str(ROOT/'EV3_RobotControl/btcomm.c'),'-lbluetooth','-o',str(executable)],check=True)
    command=[str(executable),str(work/'separated'),str(work/'overlap')]
    run5=ROOT/'calibration'/'run-5'
    if run5.is_dir(): command.append(str(run5))
    subprocess.run(command,check=True)
