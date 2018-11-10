import nextage_manipulator as nextage


def do_command(command):
    if command:
        if command.startswith('servoOn'):
            nextage.servoOn()
        elif command.startswith('servoOff'):
            nextage.servoOff()
        elif command.startswith('goInitial'):
            nextage.goInitial()
        elif command.startswith('goOff'):
            nextage.goOffPose()
        elif command.startswith('rhandOpen'):
            nextage.rhandOpen()
        elif command.startswith('rhandClose'):
            nextage.rhandClose()
        elif command.startswith('lhandOpen'):
            nextage.lhandOpen()
        elif command.startswith('lhandClose'):
            nextage.lhandClose()
        elif command.startswith('joints'):
            splitted = command.split(':')
            start = splitted[0].find('[') + 1
            end = splitted[0].find(']')
            if start < end:
                jointStr = splitted[0][start:end]
            else:
                jointStr = ''
            time = float(splitted[1])
            angles = [float(q) for q in jointStr.split(',')]
            nextage.setJointAngles(angles, time)
        else:
            print('Command Not Found : ' + command)


def do_commands(commands_str):
    for command in commands_str.split(';'):
        do_command(command.strip())
