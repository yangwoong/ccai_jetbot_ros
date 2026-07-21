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

