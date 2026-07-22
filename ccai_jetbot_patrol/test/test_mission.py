from ccai_jetbot_patrol.mission import parse_mission_command


def test_parse_json_command():
    command = parse_mission_command('{"type":"inspect","target":"entrance"}')
    assert command.type == "inspect"
    assert command.target == "entrance"


def test_parse_short_commands():
    assert parse_mission_command("patrol start").type == "patrol_start"
    assert parse_mission_command("patrol stop").type == "patrol_stop"
    assert parse_mission_command("go home").type == "go_home"
    assert parse_mission_command("inspect lobby").target == "lobby"
    assert parse_mission_command("follow person").type == "follow_person"


def test_parse_korean_and_alias_commands():
    assert parse_mission_command("상태").type == "status"
    assert parse_mission_command("시작").type == "patrol_start"
    assert parse_mission_command("정지").type == "patrol_stop"
    assert parse_mission_command("복귀").type == "go_home"
    assert parse_mission_command("점검 현관").target == "현관"
    assert parse_mission_command("상태 알려줘").type == "status"
    assert parse_mission_command("순찰 시작해").type == "patrol_start"
    assert parse_mission_command("입구를 점검하고 보고해").type == "inspect"
    assert parse_mission_command("사람 따라가").type == "follow_person"


def test_parse_llm_json_alias_command():
    command = parse_mission_command('{"command":"start_patrol"}')
    assert command.type == "patrol_start"
