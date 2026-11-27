import json
import pytest
from replay import decode, study


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    folder=tmp_path_factory.mktemp("can")
    study(folder)
    return folder,json.loads((folder/"results.json").read_text())


@pytest.mark.parametrize("run_id",["a","b"])
@pytest.mark.parametrize("mode,expected",[("staged",2.8),("warning",2.4)])
def test_brake_event(run,run_id,mode,expected):
    data=run[1][f"{run_id}_{mode}"]
    assert (data["frames"],data["first_aeb_s"],data["issues"])==(240,expected,[])


@pytest.mark.parametrize("case,ms,reasons",[
    ("drop",500,["counter_gap"]),("repeat",450,["duplicate"]),
    ("timeout",600,["timeout","counter_gap"])])
def test_faults(run,case,ms,reasons):
    assert run[1][case]["issues"]==[dict(id="500",observed_ms=ms,reason=r) for r in reasons]


def test_manual_frame_and_wrap(run):
    lines=(run[0]/"a_staged.log").read_text().splitlines()
    # 16.18 m/s -> 1618 -> LE 52 06; two 40% signals -> 28 hex;
    # level 2, rolling counter 8 -> 28 hex.
    assert lines[56]=="(2.800) simcan0 500#5206002828280000"
    assert decode(lines[15:17])[0]["issues"]==[]
    # A synthetic PID-only frame must not be counted as emergency braking.
    pid_only="(0.050) simcan0 500#5206280028010000"
    report,signals=decode([pid_only])
    assert signals[0]["OrdinaryBrake"]==0.4 and report["first_aeb_s"] is None


@pytest.mark.parametrize("end_ms,alarm", [(None, False), (11949, False), (11950, True), (12000, True)])
def test_silence_after_final_message(run,end_ms,alarm):
    lines=(run[0]/"a_staged.log").read_text().splitlines()[:-3]
    report,_=decode(lines,observation_end_ms=end_ms)
    expected=[dict(id="500",observed_ms=end_ms,deadline_ms=11950,reason="terminal_timeout")] if alarm else []
    assert report["issues"]==expected


def test_end_cannot_precede_log(run):
    lines=(run[0]/"a_staged.log").read_text().splitlines()
    with pytest.raises(ValueError,match="end"):
        decode(lines,observation_end_ms=11949)
