/*
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{ 0:IVar("omega1"), 1:IVar("theta2"), 2:IVar("theta1"), 3:INeg(InstructionI(1)), 4:IAdd(InstructionI(2), I(InstructionI(3))), 5:IFuncSin(InstructionI(4)), 6:IExp { base: I(InstructionI(0)), power: C(2.0) }, 7:IExp { base: I(InstructionI(5)), power: C(2.0) }, 8:IVar("omega1"), 9:IVar("theta2"), 10:IVar("theta1"), 11:INeg(InstructionI(9)), 12:IAdd(InstructionI(10), I(InstructionI(11))), 13:IExp { base: I(InstructionI(8)), power: C(2.0) }, 14:IFuncCos(InstructionI(12)), 15:IVar("omega2"), 16:IMul(InstructionI(13), I(InstructionI(14))), 17:IExp { base: I(InstructionI(15)), power: C(2.0) }, 18:IVar("theta2"), 19:IVar("theta1"), 20:INeg(InstructionI(18)), 21:IAdd(InstructionI(19), I(InstructionI(20))), 22:IAdd(InstructionI(16), I(InstructionI(17))), 23:IFuncCos(InstructionI(21)), 24:IMul(InstructionI(22), I(InstructionI(23))), 25:IVar("theta2"), 26:IVar("theta2"), 27:IVar("theta1"), 28:INeg(InstructionI(26)), 29:IAdd(InstructionI(27), I(InstructionI(28))), 30:IFuncSin(InstructionI(25)), 31:IFuncSin(InstructionI(29)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IMul(InstructionI(32), C(9.81)), 34:IVar("theta1"), 35:IFuncCos(InstructionI(34)), 36:IMul(InstructionI(35), C(19.62)), 37:IMul(InstructionI(6), I(InstructionI(7))), 38:INeg(InstructionI(24)), 39:IAdd(InstructionI(37), I(InstructionI(38))), 40:INeg(InstructionI(33)), 41:IAdd(InstructionI(39), I(InstructionI(40))), 42:INeg(InstructionI(36)), 43:IVar("theta2"), 44:IVar("theta1"), 45:INeg(InstructionI(43)), 46:IAdd(InstructionI(44), I(InstructionI(45))), 47:IFuncSin(InstructionI(46)), 48:IExp { base: I(InstructionI(47)), power: C(2.0) }, 49:IAdd(InstructionI(48), C(1.0)), 50:IAdd(InstructionI(41), I(InstructionI(42))), 51:IInv(InstructionI(49)), 52:IVar("omega1"), 53:INeg(InstructionI(52)), 54:IVar("omega1"), 55:IExp { base: I(InstructionI(54)), power: C(2.0) }, 56:IAdd(InstructionI(55), C(1e-10)), 57:IExp { base: I(InstructionI(56)), power: C(0.5) }, 58:IExp { base: I(InstructionI(53)), power: C(3.0) }, 59:IInv(InstructionI(57)), 60:IVar("omega1"), 61:IVar("theta2"), 62:IVar("theta1"), 63:INeg(InstructionI(61)), 64:IAdd(InstructionI(62), I(InstructionI(63))), 65:IExp { base: I(InstructionI(60)), power: C(2.0) }, 66:IFuncCos(InstructionI(64)), 67:IVar("omega2"), 68:IMul(InstructionI(65), I(InstructionI(66))), 69:IExp { base: I(InstructionI(67)), power: C(2.0) }, 70:IVar("theta2"), 71:IVar("theta1"), 72:INeg(InstructionI(70)), 73:IAdd(InstructionI(71), I(InstructionI(72))), 74:IAdd(InstructionI(68), I(InstructionI(69))), 75:IFuncSin(InstructionI(73)), 76:IMul(InstructionI(74), I(InstructionI(75))), 77:IVar("theta1"), 78:IFuncSin(InstructionI(77)), 79:IMul(InstructionI(78), C(19.62)), 80:IVar("theta2"), 81:IVar("theta2"), 82:IVar("theta1"), 83:INeg(InstructionI(81)), 84:IAdd(InstructionI(82), I(InstructionI(83))), 85:IFuncSin(InstructionI(80)), 86:IFuncCos(InstructionI(84)), 87:IMul(InstructionI(85), I(InstructionI(86))), 88:IMul(InstructionI(58), I(InstructionI(59))), 89:INeg(InstructionI(79)), 90:IAdd(InstructionI(88), I(InstructionI(89))), 91:INeg(InstructionI(76)), 92:IAdd(InstructionI(90), I(InstructionI(91))), 93:IVar("u1"), 94:IAdd(InstructionI(92), I(InstructionI(93))), 95:IMul(InstructionI(87), C(9.81)), 96:IVar("theta2"), 97:IVar("theta1"), 98:INeg(InstructionI(96)), 99:IAdd(InstructionI(97), I(InstructionI(98))), 100:IVar("theta2"), 101:IVar("theta1"), 102:INeg(InstructionI(100)), 103:IAdd(InstructionI(101), I(InstructionI(102))), 104:IVar("theta2"), 105:IVar("theta1"), 106:INeg(InstructionI(104)), 107:IAdd(InstructionI(105), I(InstructionI(106))), 108:IFuncSin(InstructionI(107)), 109:IExp { base: I(InstructionI(108)), power: C(2.0) }, 110:IAdd(InstructionI(109), C(1.0)), 111:IExp { base: I(InstructionI(110)), power: C(2.0) }, 112:IAdd(InstructionI(94), I(InstructionI(95))), 113:IFuncSin(InstructionI(99)), 114:IMul(InstructionI(112), I(InstructionI(113))), 115:IInv(InstructionI(111)), 116:IMul(InstructionI(114), I(InstructionI(115))), 117:IFuncCos(InstructionI(103)), 118:IMul(InstructionI(116), I(InstructionI(117))), 119:IMul(InstructionI(118), C(2.0)), 120:IMul(InstructionI(50), I(InstructionI(51))), 121:INeg(InstructionI(119)) } }
CompileSlab{ instrs:{ 0:IVar("omega1"), 1:IVar("omega1"), 2:IExp { base: I(InstructionI(1)), power: C(2.0) }, 3:IAdd(InstructionI(2), C(1e-10)), 4:IExp { base: I(InstructionI(3)), power: C(1.5) }, 5:IInv(InstructionI(4)), 6:IExp { base: I(InstructionI(0)), power: C(4.0) }, 7:IVar("omega1"), 8:IVar("omega1"), 9:IExp { base: I(InstructionI(8)), power: C(2.0) }, 10:IAdd(InstructionI(9), C(1e-10)), 11:IExp { base: I(InstructionI(10)), power: C(0.5) }, 12:IInv(InstructionI(11)), 13:IExp { base: I(InstructionI(7)), power: C(2.0) }, 14:IMul(InstructionI(12), I(InstructionI(13))), 15:IMul(InstructionI(14), C(3.0)), 16:IVar("theta2"), 17:IVar("theta1"), 18:INeg(InstructionI(16)), 19:IAdd(InstructionI(17), I(InstructionI(18))), 20:IVar("theta2"), 21:IVar("theta1"), 22:INeg(InstructionI(20)), 23:IAdd(InstructionI(21), I(InstructionI(22))), 24:IVar("omega1"), 25:IFuncSin(InstructionI(19)), 26:IMul(InstructionI(24), I(InstructionI(25))), 27:IFuncCos(InstructionI(23)), 28:IMul(InstructionI(26), I(InstructionI(27))), 29:IMul(InstructionI(28), C(2.0)), 30:IMul(InstructionI(5), I(InstructionI(6))), 31:INeg(InstructionI(15)), 32:IAdd(InstructionI(30), I(InstructionI(31))), 33:INeg(InstructionI(29)), 34:IVar("theta2"), 35:IVar("theta1"), 36:INeg(InstructionI(34)), 37:IAdd(InstructionI(35), I(InstructionI(36))), 38:IFuncSin(InstructionI(37)), 39:IExp { base: I(InstructionI(38)), power: C(2.0) }, 40:IAdd(InstructionI(39), C(1.0)), 41:IAdd(InstructionI(32), I(InstructionI(33))), 42:IInv(InstructionI(40)) } }
CompileSlab{ instrs:{ 0:IVar("omega1"), 1:INeg(InstructionI(0)), 2:IVar("theta2"), 3:IVar("theta1"), 4:INeg(InstructionI(2)), 5:IAdd(InstructionI(3), I(InstructionI(4))), 6:IFuncSin(InstructionI(5)), 7:IExp { base: I(InstructionI(1)), power: C(2.0) }, 8:IExp { base: I(InstructionI(6)), power: C(2.0) }, 9:IVar("omega1"), 10:IVar("theta2"), 11:IVar("theta1"), 12:INeg(InstructionI(10)), 13:IAdd(InstructionI(11), I(InstructionI(12))), 14:IExp { base: I(InstructionI(9)), power: C(2.0) }, 15:IFuncCos(InstructionI(13)), 16:IVar("omega2"), 17:IMul(InstructionI(14), I(InstructionI(15))), 18:IExp { base: I(InstructionI(16)), power: C(2.0) }, 19:IVar("theta2"), 20:IVar("theta1"), 21:INeg(InstructionI(19)), 22:IAdd(InstructionI(20), I(InstructionI(21))), 23:IAdd(InstructionI(17), I(InstructionI(18))), 24:IFuncCos(InstructionI(22)), 25:IVar("theta2"), 26:IVar("theta2"), 27:IVar("theta1"), 28:INeg(InstructionI(26)), 29:IAdd(InstructionI(27), I(InstructionI(28))), 30:IFuncSin(InstructionI(25)), 31:IFuncSin(InstructionI(29)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IVar("theta2"), 34:IVar("theta2"), 35:IVar("theta1"), 36:INeg(InstructionI(34)), 37:IAdd(InstructionI(35), I(InstructionI(36))), 38:IFuncCos(InstructionI(33)), 39:IFuncCos(InstructionI(37)), 40:IMul(InstructionI(38), I(InstructionI(39))), 41:IMul(InstructionI(7), I(InstructionI(8))), 42:IMul(InstructionI(23), I(InstructionI(24))), 43:IAdd(InstructionI(41), I(InstructionI(42))), 44:IMul(InstructionI(32), C(9.81)), 45:IAdd(InstructionI(43), I(InstructionI(44))), 46:IMul(InstructionI(40), C(9.81)), 47:IVar("theta2"), 48:IVar("theta1"), 49:INeg(InstructionI(47)), 50:IAdd(InstructionI(48), I(InstructionI(49))), 51:IFuncSin(InstructionI(50)), 52:IExp { base: I(InstructionI(51)), power: C(2.0) }, 53:IAdd(InstructionI(52), C(1.0)), 54:IAdd(InstructionI(45), I(InstructionI(46))), 55:IInv(InstructionI(53)), 56:IVar("omega1"), 57:INeg(InstructionI(56)), 58:IVar("omega1"), 59:IExp { base: I(InstructionI(58)), power: C(2.0) }, 60:IAdd(InstructionI(59), C(1e-10)), 61:IExp { base: I(InstructionI(60)), power: C(0.5) }, 62:IExp { base: I(InstructionI(57)), power: C(3.0) }, 63:IInv(InstructionI(61)), 64:IVar("omega1"), 65:IVar("theta2"), 66:IVar("theta1"), 67:INeg(InstructionI(65)), 68:IAdd(InstructionI(66), I(InstructionI(67))), 69:IExp { base: I(InstructionI(64)), power: C(2.0) }, 70:IFuncCos(InstructionI(68)), 71:IVar("omega2"), 72:IMul(InstructionI(69), I(InstructionI(70))), 73:IExp { base: I(InstructionI(71)), power: C(2.0) }, 74:IVar("theta2"), 75:IVar("theta1"), 76:INeg(InstructionI(74)), 77:IAdd(InstructionI(75), I(InstructionI(76))), 78:IAdd(InstructionI(72), I(InstructionI(73))), 79:IFuncSin(InstructionI(77)), 80:IMul(InstructionI(78), I(InstructionI(79))), 81:IVar("theta1"), 82:IFuncSin(InstructionI(81)), 83:IMul(InstructionI(82), C(19.62)), 84:IVar("theta2"), 85:IVar("theta2"), 86:IVar("theta1"), 87:INeg(InstructionI(85)), 88:IAdd(InstructionI(86), I(InstructionI(87))), 89:IFuncSin(InstructionI(84)), 90:IFuncCos(InstructionI(88)), 91:IMul(InstructionI(89), I(InstructionI(90))), 92:IMul(InstructionI(62), I(InstructionI(63))), 93:INeg(InstructionI(83)), 94:IAdd(InstructionI(92), I(InstructionI(93))), 95:INeg(InstructionI(80)), 96:IAdd(InstructionI(94), I(InstructionI(95))), 97:IVar("u1"), 98:IAdd(InstructionI(96), I(InstructionI(97))), 99:IMul(InstructionI(91), C(9.81)), 100:IVar("theta2"), 101:IVar("theta1"), 102:INeg(InstructionI(100)), 103:IAdd(InstructionI(101), I(InstructionI(102))), 104:IVar("theta2"), 105:IVar("theta1"), 106:INeg(InstructionI(104)), 107:IAdd(InstructionI(105), I(InstructionI(106))), 108:IVar("theta2"), 109:IVar("theta1"), 110:INeg(InstructionI(108)), 111:IAdd(InstructionI(109), I(InstructionI(110))), 112:IFuncSin(InstructionI(111)), 113:IExp { base: I(InstructionI(112)), power: C(2.0) }, 114:IAdd(InstructionI(113), C(1.0)), 115:IExp { base: I(InstructionI(114)), power: C(2.0) }, 116:IAdd(InstructionI(98), I(InstructionI(99))), 117:IFuncSin(InstructionI(103)), 118:IMul(InstructionI(116), I(InstructionI(117))), 119:IInv(InstructionI(115)), 120:IMul(InstructionI(118), I(InstructionI(119))), 121:IFuncCos(InstructionI(107)), 122:IMul(InstructionI(120), I(InstructionI(121))), 123:IMul(InstructionI(54), I(InstructionI(55))), 124:IMul(InstructionI(122), C(2.0)) } }
CompileSlab{ instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IVar("theta2"), 5:IVar("theta1"), 6:INeg(InstructionI(4)), 7:IAdd(InstructionI(5), I(InstructionI(6))), 8:IFuncSin(InstructionI(7)), 9:IExp { base: I(InstructionI(8)), power: C(2.0) }, 10:IAdd(InstructionI(9), C(1.0)), 11:IVar("omega2"), 12:IInv(InstructionI(10)), 13:IMul(InstructionI(11), I(InstructionI(12))), 14:IFuncSin(InstructionI(3)), 15:IMul(InstructionI(13), I(InstructionI(14))) } }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{ 0:IVar("omega1"), 1:IVar("theta2"), 2:IVar("theta1"), 3:INeg(InstructionI(1)), 4:IAdd(InstructionI(2), I(InstructionI(3))), 5:IExp { base: I(InstructionI(0)), power: C(2.0) }, 6:IFuncCos(InstructionI(4)), 7:IMul(InstructionI(5), I(InstructionI(6))), 8:IVar("omega2"), 9:IVar("theta2"), 10:IVar("theta1"), 11:INeg(InstructionI(9)), 12:IAdd(InstructionI(10), I(InstructionI(11))), 13:IFuncSin(InstructionI(12)), 14:IExp { base: I(InstructionI(8)), power: C(2.0) }, 15:IExp { base: I(InstructionI(13)), power: C(2.0) }, 16:IMul(InstructionI(14), I(InstructionI(15))), 17:IVar("omega2"), 18:IVar("theta2"), 19:IVar("theta1"), 20:INeg(InstructionI(18)), 21:IAdd(InstructionI(19), I(InstructionI(20))), 22:IFuncCos(InstructionI(21)), 23:IExp { base: I(InstructionI(17)), power: C(2.0) }, 24:IExp { base: I(InstructionI(22)), power: C(2.0) }, 25:IVar("theta1"), 26:IVar("theta2"), 27:IVar("theta1"), 28:INeg(InstructionI(26)), 29:IAdd(InstructionI(27), I(InstructionI(28))), 30:IFuncSin(InstructionI(25)), 31:IFuncSin(InstructionI(29)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IMul(InstructionI(32), C(19.62)), 34:IVar("theta1"), 35:IVar("theta2"), 36:IVar("theta1"), 37:INeg(InstructionI(35)), 38:IAdd(InstructionI(36), I(InstructionI(37))), 39:IFuncCos(InstructionI(34)), 40:IFuncCos(InstructionI(38)), 41:IMul(InstructionI(39), I(InstructionI(40))), 42:INeg(InstructionI(16)), 43:IMul(InstructionI(7), C(2.0)), 44:IAdd(InstructionI(42), I(InstructionI(43))), 45:INeg(InstructionI(33)), 46:IAdd(InstructionI(44), I(InstructionI(45))), 47:IMul(InstructionI(23), I(InstructionI(24))), 48:IAdd(InstructionI(46), I(InstructionI(47))), 49:IMul(InstructionI(41), C(19.62)), 50:IVar("theta2"), 51:IVar("theta1"), 52:INeg(InstructionI(50)), 53:IAdd(InstructionI(51), I(InstructionI(52))), 54:IFuncSin(InstructionI(53)), 55:IExp { base: I(InstructionI(54)), power: C(2.0) }, 56:IAdd(InstructionI(55), C(1.0)), 57:IAdd(InstructionI(48), I(InstructionI(49))), 58:IInv(InstructionI(56)), 59:IVar("omega1"), 60:IVar("theta2"), 61:IVar("theta1"), 62:INeg(InstructionI(60)), 63:IAdd(InstructionI(61), I(InstructionI(62))), 64:IExp { base: I(InstructionI(59)), power: C(2.0) }, 65:IFuncSin(InstructionI(63)), 66:IMul(InstructionI(64), I(InstructionI(65))), 67:IVar("omega2"), 68:IVar("omega2"), 69:IExp { base: I(InstructionI(68)), power: C(2.0) }, 70:IAdd(InstructionI(69), C(1e-10)), 71:IExp { base: I(InstructionI(70)), power: C(0.5) }, 72:IExp { base: I(InstructionI(67)), power: C(3.0) }, 73:IInv(InstructionI(71)), 74:IMul(InstructionI(72), I(InstructionI(73))), 75:IVar("omega2"), 76:IVar("theta2"), 77:IVar("theta1"), 78:INeg(InstructionI(76)), 79:IAdd(InstructionI(77), I(InstructionI(78))), 80:IVar("theta2"), 81:IVar("theta1"), 82:INeg(InstructionI(80)), 83:IAdd(InstructionI(81), I(InstructionI(82))), 84:IExp { base: I(InstructionI(75)), power: C(2.0) }, 85:IFuncSin(InstructionI(79)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IFuncCos(InstructionI(83)), 88:IVar("theta1"), 89:IVar("theta2"), 90:IVar("theta1"), 91:INeg(InstructionI(89)), 92:IAdd(InstructionI(90), I(InstructionI(91))), 93:IFuncSin(InstructionI(88)), 94:IFuncCos(InstructionI(92)), 95:IMul(InstructionI(93), I(InstructionI(94))), 96:IVar("theta2"), 97:IFuncSin(InstructionI(96)), 98:IMul(InstructionI(97), C(19.62)), 99:INeg(InstructionI(74)), 100:IMul(InstructionI(66), C(2.0)), 101:IAdd(InstructionI(99), I(InstructionI(100))), 102:IMul(InstructionI(86), I(InstructionI(87))), 103:IAdd(InstructionI(101), I(InstructionI(102))), 104:IVar("u2"), 105:IAdd(InstructionI(103), I(InstructionI(104))), 106:INeg(InstructionI(98)), 107:IAdd(InstructionI(105), I(InstructionI(106))), 108:IMul(InstructionI(95), C(19.62)), 109:IVar("theta2"), 110:IVar("theta1"), 111:INeg(InstructionI(109)), 112:IAdd(InstructionI(110), I(InstructionI(111))), 113:IVar("theta2"), 114:IVar("theta1"), 115:INeg(InstructionI(113)), 116:IAdd(InstructionI(114), I(InstructionI(115))), 117:IVar("theta2"), 118:IVar("theta1"), 119:INeg(InstructionI(117)), 120:IAdd(InstructionI(118), I(InstructionI(119))), 121:IFuncSin(InstructionI(120)), 122:IExp { base: I(InstructionI(121)), power: C(2.0) }, 123:IAdd(InstructionI(122), C(1.0)), 124:IExp { base: I(InstructionI(123)), power: C(2.0) }, 125:IAdd(InstructionI(107), I(InstructionI(108))), 126:IFuncSin(InstructionI(112)), 127:IMul(InstructionI(125), I(InstructionI(126))), 128:IInv(InstructionI(124)), 129:IMul(InstructionI(127), I(InstructionI(128))), 130:IFuncCos(InstructionI(116)), 131:IMul(InstructionI(129), I(InstructionI(130))), 132:IMul(InstructionI(131), C(2.0)), 133:IMul(InstructionI(57), I(InstructionI(58))), 134:INeg(InstructionI(132)) } }
CompileSlab{ instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IVar("theta2"), 5:IVar("theta1"), 6:INeg(InstructionI(4)), 7:IAdd(InstructionI(5), I(InstructionI(6))), 8:IFuncSin(InstructionI(7)), 9:IExp { base: I(InstructionI(8)), power: C(2.0) }, 10:IAdd(InstructionI(9), C(1.0)), 11:IVar("omega1"), 12:IInv(InstructionI(10)), 13:IMul(InstructionI(11), I(InstructionI(12))), 14:IFuncSin(InstructionI(3)), 15:IMul(InstructionI(13), I(InstructionI(14))) } }
CompileSlab{ instrs:{ 0:IVar("omega1"), 1:IVar("theta2"), 2:IVar("theta1"), 3:INeg(InstructionI(1)), 4:IAdd(InstructionI(2), I(InstructionI(3))), 5:IExp { base: I(InstructionI(0)), power: C(2.0) }, 6:IFuncCos(InstructionI(4)), 7:IMul(InstructionI(5), I(InstructionI(6))), 8:IVar("omega2"), 9:IVar("theta2"), 10:IVar("theta1"), 11:INeg(InstructionI(9)), 12:IAdd(InstructionI(10), I(InstructionI(11))), 13:IFuncSin(InstructionI(12)), 14:IExp { base: I(InstructionI(8)), power: C(2.0) }, 15:IExp { base: I(InstructionI(13)), power: C(2.0) }, 16:IVar("omega2"), 17:IVar("theta2"), 18:IVar("theta1"), 19:INeg(InstructionI(17)), 20:IAdd(InstructionI(18), I(InstructionI(19))), 21:IFuncCos(InstructionI(20)), 22:IExp { base: I(InstructionI(16)), power: C(2.0) }, 23:IExp { base: I(InstructionI(21)), power: C(2.0) }, 24:IMul(InstructionI(22), I(InstructionI(23))), 25:IVar("theta1"), 26:IVar("theta2"), 27:IVar("theta1"), 28:INeg(InstructionI(26)), 29:IAdd(InstructionI(27), I(InstructionI(28))), 30:IFuncSin(InstructionI(25)), 31:IFuncSin(InstructionI(29)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IVar("theta2"), 34:IFuncCos(InstructionI(33)), 35:IMul(InstructionI(34), C(19.62)), 36:IMul(InstructionI(7), C(-2.0)), 37:INeg(InstructionI(24)), 38:IAdd(InstructionI(36), I(InstructionI(37))), 39:IMul(InstructionI(14), I(InstructionI(15))), 40:IAdd(InstructionI(38), I(InstructionI(39))), 41:INeg(InstructionI(35)), 42:IAdd(InstructionI(40), I(InstructionI(41))), 43:IMul(InstructionI(32), C(19.62)), 44:IVar("theta2"), 45:IVar("theta1"), 46:INeg(InstructionI(44)), 47:IAdd(InstructionI(45), I(InstructionI(46))), 48:IFuncSin(InstructionI(47)), 49:IExp { base: I(InstructionI(48)), power: C(2.0) }, 50:IAdd(InstructionI(49), C(1.0)), 51:IAdd(InstructionI(42), I(InstructionI(43))), 52:IInv(InstructionI(50)), 53:IVar("omega1"), 54:IVar("theta2"), 55:IVar("theta1"), 56:INeg(InstructionI(54)), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IExp { base: I(InstructionI(53)), power: C(2.0) }, 59:IFuncSin(InstructionI(57)), 60:IMul(InstructionI(58), I(InstructionI(59))), 61:IVar("omega2"), 62:IVar("omega2"), 63:IExp { base: I(InstructionI(62)), power: C(2.0) }, 64:IAdd(InstructionI(63), C(1e-10)), 65:IExp { base: I(InstructionI(64)), power: C(0.5) }, 66:IExp { base: I(InstructionI(61)), power: C(3.0) }, 67:IInv(InstructionI(65)), 68:IMul(InstructionI(66), I(InstructionI(67))), 69:IVar("omega2"), 70:IVar("theta2"), 71:IVar("theta1"), 72:INeg(InstructionI(70)), 73:IAdd(InstructionI(71), I(InstructionI(72))), 74:IVar("theta2"), 75:IVar("theta1"), 76:INeg(InstructionI(74)), 77:IAdd(InstructionI(75), I(InstructionI(76))), 78:IExp { base: I(InstructionI(69)), power: C(2.0) }, 79:IFuncSin(InstructionI(73)), 80:IMul(InstructionI(78), I(InstructionI(79))), 81:IFuncCos(InstructionI(77)), 82:IVar("theta1"), 83:IVar("theta2"), 84:IVar("theta1"), 85:INeg(InstructionI(83)), 86:IAdd(InstructionI(84), I(InstructionI(85))), 87:IFuncSin(InstructionI(82)), 88:IFuncCos(InstructionI(86)), 89:IMul(InstructionI(87), I(InstructionI(88))), 90:IVar("theta2"), 91:IFuncSin(InstructionI(90)), 92:IMul(InstructionI(91), C(19.62)), 93:INeg(InstructionI(68)), 94:IMul(InstructionI(60), C(2.0)), 95:IAdd(InstructionI(93), I(InstructionI(94))), 96:IMul(InstructionI(80), I(InstructionI(81))), 97:IAdd(InstructionI(95), I(InstructionI(96))), 98:IVar("u2"), 99:IAdd(InstructionI(97), I(InstructionI(98))), 100:INeg(InstructionI(92)), 101:IAdd(InstructionI(99), I(InstructionI(100))), 102:IMul(InstructionI(89), C(19.62)), 103:IVar("theta2"), 104:IVar("theta1"), 105:INeg(InstructionI(103)), 106:IAdd(InstructionI(104), I(InstructionI(105))), 107:IVar("theta2"), 108:IVar("theta1"), 109:INeg(InstructionI(107)), 110:IAdd(InstructionI(108), I(InstructionI(109))), 111:IVar("theta2"), 112:IVar("theta1"), 113:INeg(InstructionI(111)), 114:IAdd(InstructionI(112), I(InstructionI(113))), 115:IFuncSin(InstructionI(114)), 116:IExp { base: I(InstructionI(115)), power: C(2.0) }, 117:IAdd(InstructionI(116), C(1.0)), 118:IExp { base: I(InstructionI(117)), power: C(2.0) }, 119:IAdd(InstructionI(101), I(InstructionI(102))), 120:IFuncSin(InstructionI(106)), 121:IMul(InstructionI(119), I(InstructionI(120))), 122:IInv(InstructionI(118)), 123:IMul(InstructionI(121), I(InstructionI(122))), 124:IFuncCos(InstructionI(110)), 125:IMul(InstructionI(123), I(InstructionI(124))), 126:IMul(InstructionI(51), I(InstructionI(52))), 127:IMul(InstructionI(125), C(2.0)) } }
CompileSlab{ instrs:{ 0:IVar("omega2"), 1:IVar("omega2"), 2:IExp { base: I(InstructionI(1)), power: C(2.0) }, 3:IAdd(InstructionI(2), C(1e-10)), 4:IExp { base: I(InstructionI(3)), power: C(1.5) }, 5:IInv(InstructionI(4)), 6:IExp { base: I(InstructionI(0)), power: C(4.0) }, 7:IVar("omega2"), 8:IVar("omega2"), 9:IExp { base: I(InstructionI(8)), power: C(2.0) }, 10:IAdd(InstructionI(9), C(1e-10)), 11:IExp { base: I(InstructionI(10)), power: C(0.5) }, 12:IInv(InstructionI(11)), 13:IExp { base: I(InstructionI(7)), power: C(2.0) }, 14:IMul(InstructionI(12), I(InstructionI(13))), 15:IMul(InstructionI(14), C(3.0)), 16:IVar("theta2"), 17:IVar("theta1"), 18:INeg(InstructionI(16)), 19:IAdd(InstructionI(17), I(InstructionI(18))), 20:IVar("theta2"), 21:IVar("theta1"), 22:INeg(InstructionI(20)), 23:IAdd(InstructionI(21), I(InstructionI(22))), 24:IVar("omega2"), 25:IFuncSin(InstructionI(19)), 26:IMul(InstructionI(24), I(InstructionI(25))), 27:IFuncCos(InstructionI(23)), 28:IMul(InstructionI(26), I(InstructionI(27))), 29:INeg(InstructionI(15)), 30:IMul(InstructionI(5), I(InstructionI(6))), 31:IAdd(InstructionI(29), I(InstructionI(30))), 32:IMul(InstructionI(28), C(2.0)), 33:IVar("theta2"), 34:IVar("theta1"), 35:INeg(InstructionI(33)), 36:IAdd(InstructionI(34), I(InstructionI(35))), 37:IFuncSin(InstructionI(36)), 38:IExp { base: I(InstructionI(37)), power: C(2.0) }, 39:IAdd(InstructionI(38), C(1.0)), 40:IAdd(InstructionI(31), I(InstructionI(32))), 41:IInv(InstructionI(39)) } }



  [[(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(1.0), Slab{ exprs:{ 0:Expression { first: EConstant(1.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} })], [(IAdd(InstructionI(120), I(InstructionI(121))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(1)))), ExprPair(EAdd, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 4:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 6:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 7:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(0)))), ExprPair(EExp, EConstant(2.0)), ExprPair(ESub, EUnaryOp(EParentheses(ExpressionI(2)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(3)))), ExprPair(ESub, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(4)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(5)))), ExprPair(ESub, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(6))))] }, 8:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 9:Expression { first: EStdFunc(EFuncSin(ExpressionI(8))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 10:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 11:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 12:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(11)))), ExprPair(EAdd, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0))] }, 13:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 14:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 15:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 16:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 17:Expression { first: EUnaryOp(ENeg(ValueI(0))), pairs: [ExprPair(EExp, EConstant(3.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(10)))), ExprPair(EExp, EConstant(0.5)), ExprPair(EAdd, EStdFunc(EVar("u1"))), ExprPair(ESub, EUnaryOp(EParentheses(ExpressionI(12)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(13)))), ExprPair(ESub, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(14)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(15)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(16))))] }, 18:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 19:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 20:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 21:Expression { first: EStdFunc(EFuncSin(ExpressionI(20))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 22:Expression { first: EUnaryOp(EParentheses(ExpressionI(7))), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(9)))), ExprPair(ESub, EConstant(2.0)), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(17)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(18)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(19)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(21)))), ExprPair(EExp, EConstant(2.0))] } }, vals:{ 0:EStdFunc(EVar("omega1")) }, instrs:{ 0:IVar("omega1"), 1:IVar("theta2"), 2:IVar("theta1"), 3:INeg(InstructionI(1)), 4:IAdd(InstructionI(2), I(InstructionI(3))), 5:IFuncSin(InstructionI(4)), 6:IExp { base: I(InstructionI(0)), power: C(2.0) }, 7:IExp { base: I(InstructionI(5)), power: C(2.0) }, 8:IVar("omega1"), 9:IVar("theta2"), 10:IVar("theta1"), 11:INeg(InstructionI(9)), 12:IAdd(InstructionI(10), I(InstructionI(11))), 13:IExp { base: I(InstructionI(8)), power: C(2.0) }, 14:IFuncCos(InstructionI(12)), 15:IVar("omega2"), 16:IMul(InstructionI(13), I(InstructionI(14))), 17:IExp { base: I(InstructionI(15)), power: C(2.0) }, 18:IVar("theta2"), 19:IVar("theta1"), 20:INeg(InstructionI(18)), 21:IAdd(InstructionI(19), I(InstructionI(20))), 22:IAdd(InstructionI(16), I(InstructionI(17))), 23:IFuncCos(InstructionI(21)), 24:IMul(InstructionI(22), I(InstructionI(23))), 25:IVar("theta2"), 26:IVar("theta2"), 27:IVar("theta1"), 28:INeg(InstructionI(26)), 29:IAdd(InstructionI(27), I(InstructionI(28))), 30:IFuncSin(InstructionI(25)), 31:IFuncSin(InstructionI(29)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IMul(InstructionI(32), C(9.81)), 34:IVar("theta1"), 35:IFuncCos(InstructionI(34)), 36:IMul(InstructionI(35), C(19.62)), 37:IMul(InstructionI(6), I(InstructionI(7))), 38:INeg(InstructionI(24)), 39:IAdd(InstructionI(37), I(InstructionI(38))), 40:INeg(InstructionI(33)), 41:IAdd(InstructionI(39), I(InstructionI(40))), 42:INeg(InstructionI(36)), 43:IVar("theta2"), 44:IVar("theta1"), 45:INeg(InstructionI(43)), 46:IAdd(InstructionI(44), I(InstructionI(45))), 47:IFuncSin(InstructionI(46)), 48:IExp { base: I(InstructionI(47)), power: C(2.0) }, 49:IAdd(InstructionI(48), C(1.0)), 50:IAdd(InstructionI(41), I(InstructionI(42))), 51:IInv(InstructionI(49)), 52:IVar("omega1"), 53:INeg(InstructionI(52)), 54:IVar("omega1"), 55:IExp { base: I(InstructionI(54)), power: C(2.0) }, 56:IAdd(InstructionI(55), C(1e-10)), 57:IExp { base: I(InstructionI(56)), power: C(0.5) }, 58:IExp { base: I(InstructionI(53)), power: C(3.0) }, 59:IInv(InstructionI(57)), 60:IVar("omega1"), 61:IVar("theta2"), 62:IVar("theta1"), 63:INeg(InstructionI(61)), 64:IAdd(InstructionI(62), I(InstructionI(63))), 65:IExp { base: I(InstructionI(60)), power: C(2.0) }, 66:IFuncCos(InstructionI(64)), 67:IVar("omega2"), 68:IMul(InstructionI(65), I(InstructionI(66))), 69:IExp { base: I(InstructionI(67)), power: C(2.0) }, 70:IVar("theta2"), 71:IVar("theta1"), 72:INeg(InstructionI(70)), 73:IAdd(InstructionI(71), I(InstructionI(72))), 74:IAdd(InstructionI(68), I(InstructionI(69))), 75:IFuncSin(InstructionI(73)), 76:IMul(InstructionI(74), I(InstructionI(75))), 77:IVar("theta1"), 78:IFuncSin(InstructionI(77)), 79:IMul(InstructionI(78), C(19.62)), 80:IVar("theta2"), 81:IVar("theta2"), 82:IVar("theta1"), 83:INeg(InstructionI(81)), 84:IAdd(InstructionI(82), I(InstructionI(83))), 85:IFuncSin(InstructionI(80)), 86:IFuncCos(InstructionI(84)), 87:IMul(InstructionI(85), I(InstructionI(86))), 88:IMul(InstructionI(58), I(InstructionI(59))), 89:INeg(InstructionI(79)), 90:IAdd(InstructionI(88), I(InstructionI(89))), 91:INeg(InstructionI(76)), 92:IAdd(InstructionI(90), I(InstructionI(91))), 93:IVar("u1"), 94:IAdd(InstructionI(92), I(InstructionI(93))), 95:IMul(InstructionI(87), C(9.81)), 96:IVar("theta2"), 97:IVar("theta1"), 98:INeg(InstructionI(96)), 99:IAdd(InstructionI(97), I(InstructionI(98))), 100:IVar("theta2"), 101:IVar("theta1"), 102:INeg(InstructionI(100)), 103:IAdd(InstructionI(101), I(InstructionI(102))), 104:IVar("theta2"), 105:IVar("theta1"), 106:INeg(InstructionI(104)), 107:IAdd(InstructionI(105), I(InstructionI(106))), 108:IFuncSin(InstructionI(107)), 109:IExp { base: I(InstructionI(108)), power: C(2.0) }, 110:IAdd(InstructionI(109), C(1.0)), 111:IExp { base: I(InstructionI(110)), power: C(2.0) }, 112:IAdd(InstructionI(94), I(InstructionI(95))), 113:IFuncSin(InstructionI(99)), 114:IMul(InstructionI(112), I(InstructionI(113))), 115:IInv(InstructionI(111)), 116:IMul(InstructionI(114), I(InstructionI(115))), 117:IFuncCos(InstructionI(103)), 118:IMul(InstructionI(116), I(InstructionI(117))), 119:IMul(InstructionI(118), C(2.0)), 120:IMul(InstructionI(50), I(InstructionI(51))), 121:INeg(InstructionI(119)) } }), (IMul(InstructionI(41), I(InstructionI(42))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 1:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 2:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 4:Expression { first: EConstant(1.0), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(4.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(0)))), ExprPair(EExp, EConstant(1.5)), ExprPair(ESub, EConstant(3.0)), ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(1)))), ExprPair(EExp, EConstant(0.5)), ExprPair(ESub, EConstant(2.0)), ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(2)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(3))))] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 6:Expression { first: EStdFunc(EFuncSin(ExpressionI(5))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 7:Expression { first: EUnaryOp(EParentheses(ExpressionI(4))), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(6))))] } }, vals:{}, instrs:{ 0:IVar("omega1"), 1:IVar("omega1"), 2:IExp { base: I(InstructionI(1)), power: C(2.0) }, 3:IAdd(InstructionI(2), C(1e-10)), 4:IExp { base: I(InstructionI(3)), power: C(1.5) }, 5:IInv(InstructionI(4)), 6:IExp { base: I(InstructionI(0)), power: C(4.0) }, 7:IVar("omega1"), 8:IVar("omega1"), 9:IExp { base: I(InstructionI(8)), power: C(2.0) }, 10:IAdd(InstructionI(9), C(1e-10)), 11:IExp { base: I(InstructionI(10)), power: C(0.5) }, 12:IInv(InstructionI(11)), 13:IExp { base: I(InstructionI(7)), power: C(2.0) }, 14:IMul(InstructionI(12), I(InstructionI(13))), 15:IMul(InstructionI(14), C(3.0)), 16:IVar("theta2"), 17:IVar("theta1"), 18:INeg(InstructionI(16)), 19:IAdd(InstructionI(17), I(InstructionI(18))), 20:IVar("theta2"), 21:IVar("theta1"), 22:INeg(InstructionI(20)), 23:IAdd(InstructionI(21), I(InstructionI(22))), 24:IVar("omega1"), 25:IFuncSin(InstructionI(19)), 26:IMul(InstructionI(24), I(InstructionI(25))), 27:IFuncCos(InstructionI(23)), 28:IMul(InstructionI(26), I(InstructionI(27))), 29:IMul(InstructionI(28), C(2.0)), 30:IMul(InstructionI(5), I(InstructionI(6))), 31:INeg(InstructionI(15)), 32:IAdd(InstructionI(30), I(InstructionI(31))), 33:INeg(InstructionI(29)), 34:IVar("theta2"), 35:IVar("theta1"), 36:INeg(InstructionI(34)), 37:IAdd(InstructionI(35), I(InstructionI(36))), 38:IFuncSin(InstructionI(37)), 39:IExp { base: I(InstructionI(38)), power: C(2.0) }, 40:IAdd(InstructionI(39), C(1.0)), 41:IAdd(InstructionI(32), I(InstructionI(33))), 42:IInv(InstructionI(40)) } }), (IAdd(InstructionI(123), I(InstructionI(124))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(1)))), ExprPair(EAdd, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 4:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 6:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 7:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 8:Expression { first: EUnaryOp(ENeg(ValueI(0))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(0)))), ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EUnaryOp(EParentheses(ExpressionI(2)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(3)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(4)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(5)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(6)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(7))))] }, 9:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 10:Expression { first: EStdFunc(EFuncSin(ExpressionI(9))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 11:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 12:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 13:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(12)))), ExprPair(EAdd, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0))] }, 14:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 15:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 16:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 17:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 18:Expression { first: EUnaryOp(ENeg(ValueI(1))), pairs: [ExprPair(EExp, EConstant(3.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(11)))), ExprPair(EExp, EConstant(0.5)), ExprPair(EAdd, EStdFunc(EVar("u1"))), ExprPair(ESub, EUnaryOp(EParentheses(ExpressionI(13)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(14)))), ExprPair(ESub, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(15)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(16)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(17))))] }, 19:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 20:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 21:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 22:Expression { first: EStdFunc(EFuncSin(ExpressionI(21))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 23:Expression { first: EUnaryOp(EParentheses(ExpressionI(8))), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(10)))), ExprPair(EAdd, EConstant(2.0)), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(18)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(19)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(20)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(22)))), ExprPair(EExp, EConstant(2.0))] } }, vals:{ 0:EStdFunc(EVar("omega1")), 1:EStdFunc(EVar("omega1")) }, instrs:{ 0:IVar("omega1"), 1:INeg(InstructionI(0)), 2:IVar("theta2"), 3:IVar("theta1"), 4:INeg(InstructionI(2)), 5:IAdd(InstructionI(3), I(InstructionI(4))), 6:IFuncSin(InstructionI(5)), 7:IExp { base: I(InstructionI(1)), power: C(2.0) }, 8:IExp { base: I(InstructionI(6)), power: C(2.0) }, 9:IVar("omega1"), 10:IVar("theta2"), 11:IVar("theta1"), 12:INeg(InstructionI(10)), 13:IAdd(InstructionI(11), I(InstructionI(12))), 14:IExp { base: I(InstructionI(9)), power: C(2.0) }, 15:IFuncCos(InstructionI(13)), 16:IVar("omega2"), 17:IMul(InstructionI(14), I(InstructionI(15))), 18:IExp { base: I(InstructionI(16)), power: C(2.0) }, 19:IVar("theta2"), 20:IVar("theta1"), 21:INeg(InstructionI(19)), 22:IAdd(InstructionI(20), I(InstructionI(21))), 23:IAdd(InstructionI(17), I(InstructionI(18))), 24:IFuncCos(InstructionI(22)), 25:IVar("theta2"), 26:IVar("theta2"), 27:IVar("theta1"), 28:INeg(InstructionI(26)), 29:IAdd(InstructionI(27), I(InstructionI(28))), 30:IFuncSin(InstructionI(25)), 31:IFuncSin(InstructionI(29)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IVar("theta2"), 34:IVar("theta2"), 35:IVar("theta1"), 36:INeg(InstructionI(34)), 37:IAdd(InstructionI(35), I(InstructionI(36))), 38:IFuncCos(InstructionI(33)), 39:IFuncCos(InstructionI(37)), 40:IMul(InstructionI(38), I(InstructionI(39))), 41:IMul(InstructionI(7), I(InstructionI(8))), 42:IMul(InstructionI(23), I(InstructionI(24))), 43:IAdd(InstructionI(41), I(InstructionI(42))), 44:IMul(InstructionI(32), C(9.81)), 45:IAdd(InstructionI(43), I(InstructionI(44))), 46:IMul(InstructionI(40), C(9.81)), 47:IVar("theta2"), 48:IVar("theta1"), 49:INeg(InstructionI(47)), 50:IAdd(InstructionI(48), I(InstructionI(49))), 51:IFuncSin(InstructionI(50)), 52:IExp { base: I(InstructionI(51)), power: C(2.0) }, 53:IAdd(InstructionI(52), C(1.0)), 54:IAdd(InstructionI(45), I(InstructionI(46))), 55:IInv(InstructionI(53)), 56:IVar("omega1"), 57:INeg(InstructionI(56)), 58:IVar("omega1"), 59:IExp { base: I(InstructionI(58)), power: C(2.0) }, 60:IAdd(InstructionI(59), C(1e-10)), 61:IExp { base: I(InstructionI(60)), power: C(0.5) }, 62:IExp { base: I(InstructionI(57)), power: C(3.0) }, 63:IInv(InstructionI(61)), 64:IVar("omega1"), 65:IVar("theta2"), 66:IVar("theta1"), 67:INeg(InstructionI(65)), 68:IAdd(InstructionI(66), I(InstructionI(67))), 69:IExp { base: I(InstructionI(64)), power: C(2.0) }, 70:IFuncCos(InstructionI(68)), 71:IVar("omega2"), 72:IMul(InstructionI(69), I(InstructionI(70))), 73:IExp { base: I(InstructionI(71)), power: C(2.0) }, 74:IVar("theta2"), 75:IVar("theta1"), 76:INeg(InstructionI(74)), 77:IAdd(InstructionI(75), I(InstructionI(76))), 78:IAdd(InstructionI(72), I(InstructionI(73))), 79:IFuncSin(InstructionI(77)), 80:IMul(InstructionI(78), I(InstructionI(79))), 81:IVar("theta1"), 82:IFuncSin(InstructionI(81)), 83:IMul(InstructionI(82), C(19.62)), 84:IVar("theta2"), 85:IVar("theta2"), 86:IVar("theta1"), 87:INeg(InstructionI(85)), 88:IAdd(InstructionI(86), I(InstructionI(87))), 89:IFuncSin(InstructionI(84)), 90:IFuncCos(InstructionI(88)), 91:IMul(InstructionI(89), I(InstructionI(90))), 92:IMul(InstructionI(62), I(InstructionI(63))), 93:INeg(InstructionI(83)), 94:IAdd(InstructionI(92), I(InstructionI(93))), 95:INeg(InstructionI(80)), 96:IAdd(InstructionI(94), I(InstructionI(95))), 97:IVar("u1"), 98:IAdd(InstructionI(96), I(InstructionI(97))), 99:IMul(InstructionI(91), C(9.81)), 100:IVar("theta2"), 101:IVar("theta1"), 102:INeg(InstructionI(100)), 103:IAdd(InstructionI(101), I(InstructionI(102))), 104:IVar("theta2"), 105:IVar("theta1"), 106:INeg(InstructionI(104)), 107:IAdd(InstructionI(105), I(InstructionI(106))), 108:IVar("theta2"), 109:IVar("theta1"), 110:INeg(InstructionI(108)), 111:IAdd(InstructionI(109), I(InstructionI(110))), 112:IFuncSin(InstructionI(111)), 113:IExp { base: I(InstructionI(112)), power: C(2.0) }, 114:IAdd(InstructionI(113), C(1.0)), 115:IExp { base: I(InstructionI(114)), power: C(2.0) }, 116:IAdd(InstructionI(98), I(InstructionI(99))), 117:IFuncSin(InstructionI(103)), 118:IMul(InstructionI(116), I(InstructionI(117))), 119:IInv(InstructionI(115)), 120:IMul(InstructionI(118), I(InstructionI(119))), 121:IFuncCos(InstructionI(107)), 122:IMul(InstructionI(120), I(InstructionI(121))), 123:IMul(InstructionI(54), I(InstructionI(55))), 124:IMul(InstructionI(122), C(2.0)) } }), (IMul(InstructionI(15), C(-2.0)), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EFuncSin(ExpressionI(1))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 3:Expression { first: EConstant(-2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(0)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(2))))] } }, vals:{}, instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IVar("theta2"), 5:IVar("theta1"), 6:INeg(InstructionI(4)), 7:IAdd(InstructionI(5), I(InstructionI(6))), 8:IFuncSin(InstructionI(7)), 9:IExp { base: I(InstructionI(8)), power: C(2.0) }, 10:IAdd(InstructionI(9), C(1.0)), 11:IVar("omega2"), 12:IInv(InstructionI(10)), 13:IMul(InstructionI(11), I(InstructionI(12))), 14:IFuncSin(InstructionI(3)), 15:IMul(InstructionI(13), I(InstructionI(14))) } })], [(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(1.0), Slab{ exprs:{ 0:Expression { first: EConstant(1.0), pairs: [] } }, vals:{}, instrs:{} })], [(IAdd(InstructionI(133), I(InstructionI(134))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 4:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 6:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 7:Expression { first: EConstant(2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(0)))), ExprPair(ESub, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(1)))), ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(2)))), ExprPair(EExp, EConstant(2.0)), ExprPair(ESub, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(3)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(4)))), ExprPair(EAdd, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(5)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(6))))] }, 8:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 9:Expression { first: EStdFunc(EFuncSin(ExpressionI(8))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 10:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 11:Expression { first: EStdFunc(EVar("omega2")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 12:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 13:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 14:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 15:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 16:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 17:Expression { first: EConstant(2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(10)))), ExprPair(ESub, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(3.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(11)))), ExprPair(EExp, EConstant(0.5)), ExprPair(EAdd, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(12)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(13)))), ExprPair(EAdd, EStdFunc(EVar("u2"))), ExprPair(EAdd, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(14)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(15)))), ExprPair(ESub, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(16))))] }, 18:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 19:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 20:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 21:Expression { first: EStdFunc(EFuncSin(ExpressionI(20))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 22:Expression { first: EUnaryOp(EParentheses(ExpressionI(7))), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(9)))), ExprPair(ESub, EConstant(2.0)), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(17)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(18)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(19)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(21)))), ExprPair(EExp, EConstant(2.0))] } }, vals:{}, instrs:{ 0:IVar("omega1"), 1:IVar("theta2"), 2:IVar("theta1"), 3:INeg(InstructionI(1)), 4:IAdd(InstructionI(2), I(InstructionI(3))), 5:IExp { base: I(InstructionI(0)), power: C(2.0) }, 6:IFuncCos(InstructionI(4)), 7:IMul(InstructionI(5), I(InstructionI(6))), 8:IVar("omega2"), 9:IVar("theta2"), 10:IVar("theta1"), 11:INeg(InstructionI(9)), 12:IAdd(InstructionI(10), I(InstructionI(11))), 13:IFuncSin(InstructionI(12)), 14:IExp { base: I(InstructionI(8)), power: C(2.0) }, 15:IExp { base: I(InstructionI(13)), power: C(2.0) }, 16:IMul(InstructionI(14), I(InstructionI(15))), 17:IVar("omega2"), 18:IVar("theta2"), 19:IVar("theta1"), 20:INeg(InstructionI(18)), 21:IAdd(InstructionI(19), I(InstructionI(20))), 22:IFuncCos(InstructionI(21)), 23:IExp { base: I(InstructionI(17)), power: C(2.0) }, 24:IExp { base: I(InstructionI(22)), power: C(2.0) }, 25:IVar("theta1"), 26:IVar("theta2"), 27:IVar("theta1"), 28:INeg(InstructionI(26)), 29:IAdd(InstructionI(27), I(InstructionI(28))), 30:IFuncSin(InstructionI(25)), 31:IFuncSin(InstructionI(29)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IMul(InstructionI(32), C(19.62)), 34:IVar("theta1"), 35:IVar("theta2"), 36:IVar("theta1"), 37:INeg(InstructionI(35)), 38:IAdd(InstructionI(36), I(InstructionI(37))), 39:IFuncCos(InstructionI(34)), 40:IFuncCos(InstructionI(38)), 41:IMul(InstructionI(39), I(InstructionI(40))), 42:INeg(InstructionI(16)), 43:IMul(InstructionI(7), C(2.0)), 44:IAdd(InstructionI(42), I(InstructionI(43))), 45:INeg(InstructionI(33)), 46:IAdd(InstructionI(44), I(InstructionI(45))), 47:IMul(InstructionI(23), I(InstructionI(24))), 48:IAdd(InstructionI(46), I(InstructionI(47))), 49:IMul(InstructionI(41), C(19.62)), 50:IVar("theta2"), 51:IVar("theta1"), 52:INeg(InstructionI(50)), 53:IAdd(InstructionI(51), I(InstructionI(52))), 54:IFuncSin(InstructionI(53)), 55:IExp { base: I(InstructionI(54)), power: C(2.0) }, 56:IAdd(InstructionI(55), C(1.0)), 57:IAdd(InstructionI(48), I(InstructionI(49))), 58:IInv(InstructionI(56)), 59:IVar("omega1"), 60:IVar("theta2"), 61:IVar("theta1"), 62:INeg(InstructionI(60)), 63:IAdd(InstructionI(61), I(InstructionI(62))), 64:IExp { base: I(InstructionI(59)), power: C(2.0) }, 65:IFuncSin(InstructionI(63)), 66:IMul(InstructionI(64), I(InstructionI(65))), 67:IVar("omega2"), 68:IVar("omega2"), 69:IExp { base: I(InstructionI(68)), power: C(2.0) }, 70:IAdd(InstructionI(69), C(1e-10)), 71:IExp { base: I(InstructionI(70)), power: C(0.5) }, 72:IExp { base: I(InstructionI(67)), power: C(3.0) }, 73:IInv(InstructionI(71)), 74:IMul(InstructionI(72), I(InstructionI(73))), 75:IVar("omega2"), 76:IVar("theta2"), 77:IVar("theta1"), 78:INeg(InstructionI(76)), 79:IAdd(InstructionI(77), I(InstructionI(78))), 80:IVar("theta2"), 81:IVar("theta1"), 82:INeg(InstructionI(80)), 83:IAdd(InstructionI(81), I(InstructionI(82))), 84:IExp { base: I(InstructionI(75)), power: C(2.0) }, 85:IFuncSin(InstructionI(79)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IFuncCos(InstructionI(83)), 88:IVar("theta1"), 89:IVar("theta2"), 90:IVar("theta1"), 91:INeg(InstructionI(89)), 92:IAdd(InstructionI(90), I(InstructionI(91))), 93:IFuncSin(InstructionI(88)), 94:IFuncCos(InstructionI(92)), 95:IMul(InstructionI(93), I(InstructionI(94))), 96:IVar("theta2"), 97:IFuncSin(InstructionI(96)), 98:IMul(InstructionI(97), C(19.62)), 99:INeg(InstructionI(74)), 100:IMul(InstructionI(66), C(2.0)), 101:IAdd(InstructionI(99), I(InstructionI(100))), 102:IMul(InstructionI(86), I(InstructionI(87))), 103:IAdd(InstructionI(101), I(InstructionI(102))), 104:IVar("u2"), 105:IAdd(InstructionI(103), I(InstructionI(104))), 106:INeg(InstructionI(98)), 107:IAdd(InstructionI(105), I(InstructionI(106))), 108:IMul(InstructionI(95), C(19.62)), 109:IVar("theta2"), 110:IVar("theta1"), 111:INeg(InstructionI(109)), 112:IAdd(InstructionI(110), I(InstructionI(111))), 113:IVar("theta2"), 114:IVar("theta1"), 115:INeg(InstructionI(113)), 116:IAdd(InstructionI(114), I(InstructionI(115))), 117:IVar("theta2"), 118:IVar("theta1"), 119:INeg(InstructionI(117)), 120:IAdd(InstructionI(118), I(InstructionI(119))), 121:IFuncSin(InstructionI(120)), 122:IExp { base: I(InstructionI(121)), power: C(2.0) }, 123:IAdd(InstructionI(122), C(1.0)), 124:IExp { base: I(InstructionI(123)), power: C(2.0) }, 125:IAdd(InstructionI(107), I(InstructionI(108))), 126:IFuncSin(InstructionI(112)), 127:IMul(InstructionI(125), I(InstructionI(126))), 128:IInv(InstructionI(124)), 129:IMul(InstructionI(127), I(InstructionI(128))), 130:IFuncCos(InstructionI(116)), 131:IMul(InstructionI(129), I(InstructionI(130))), 132:IMul(InstructionI(131), C(2.0)), 133:IMul(InstructionI(57), I(InstructionI(58))), 134:INeg(InstructionI(132)) } }), (IMul(InstructionI(15), C(4.0)), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EFuncSin(ExpressionI(1))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 3:Expression { first: EConstant(4.0), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(0)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(2))))] } }, vals:{}, instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IVar("theta2"), 5:IVar("theta1"), 6:INeg(InstructionI(4)), 7:IAdd(InstructionI(5), I(InstructionI(6))), 8:IFuncSin(InstructionI(7)), 9:IExp { base: I(InstructionI(8)), power: C(2.0) }, 10:IAdd(InstructionI(9), C(1.0)), 11:IVar("omega1"), 12:IInv(InstructionI(10)), 13:IMul(InstructionI(11), I(InstructionI(12))), 14:IFuncSin(InstructionI(3)), 15:IMul(InstructionI(13), I(InstructionI(14))) } }), (IAdd(InstructionI(126), I(InstructionI(127))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 4:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 5:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 6:Expression { first: EConstant(-2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(0)))), ExprPair(EAdd, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(1)))), ExprPair(EExp, EConstant(2.0)), ExprPair(ESub, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(2)))), ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(3)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(4)))), ExprPair(ESub, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(5))))] }, 7:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 8:Expression { first: EStdFunc(EFuncSin(ExpressionI(7))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 9:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 10:Expression { first: EStdFunc(EVar("omega2")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 11:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 12:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 13:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 14:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 15:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 16:Expression { first: EConstant(2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(9)))), ExprPair(ESub, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(3.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(10)))), ExprPair(EExp, EConstant(0.5)), ExprPair(EAdd, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(11)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(12)))), ExprPair(EAdd, EStdFunc(EVar("u2"))), ExprPair(EAdd, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(13)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(14)))), ExprPair(ESub, EConstant(19.62)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(15))))] }, 17:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 18:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 19:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 20:Expression { first: EStdFunc(EFuncSin(ExpressionI(19))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 21:Expression { first: EUnaryOp(EParentheses(ExpressionI(6))), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(8)))), ExprPair(EAdd, EConstant(2.0)), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(16)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(17)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(18)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(20)))), ExprPair(EExp, EConstant(2.0))] } }, vals:{}, instrs:{ 0:IVar("omega1"), 1:IVar("theta2"), 2:IVar("theta1"), 3:INeg(InstructionI(1)), 4:IAdd(InstructionI(2), I(InstructionI(3))), 5:IExp { base: I(InstructionI(0)), power: C(2.0) }, 6:IFuncCos(InstructionI(4)), 7:IMul(InstructionI(5), I(InstructionI(6))), 8:IVar("omega2"), 9:IVar("theta2"), 10:IVar("theta1"), 11:INeg(InstructionI(9)), 12:IAdd(InstructionI(10), I(InstructionI(11))), 13:IFuncSin(InstructionI(12)), 14:IExp { base: I(InstructionI(8)), power: C(2.0) }, 15:IExp { base: I(InstructionI(13)), power: C(2.0) }, 16:IVar("omega2"), 17:IVar("theta2"), 18:IVar("theta1"), 19:INeg(InstructionI(17)), 20:IAdd(InstructionI(18), I(InstructionI(19))), 21:IFuncCos(InstructionI(20)), 22:IExp { base: I(InstructionI(16)), power: C(2.0) }, 23:IExp { base: I(InstructionI(21)), power: C(2.0) }, 24:IMul(InstructionI(22), I(InstructionI(23))), 25:IVar("theta1"), 26:IVar("theta2"), 27:IVar("theta1"), 28:INeg(InstructionI(26)), 29:IAdd(InstructionI(27), I(InstructionI(28))), 30:IFuncSin(InstructionI(25)), 31:IFuncSin(InstructionI(29)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IVar("theta2"), 34:IFuncCos(InstructionI(33)), 35:IMul(InstructionI(34), C(19.62)), 36:IMul(InstructionI(7), C(-2.0)), 37:INeg(InstructionI(24)), 38:IAdd(InstructionI(36), I(InstructionI(37))), 39:IMul(InstructionI(14), I(InstructionI(15))), 40:IAdd(InstructionI(38), I(InstructionI(39))), 41:INeg(InstructionI(35)), 42:IAdd(InstructionI(40), I(InstructionI(41))), 43:IMul(InstructionI(32), C(19.62)), 44:IVar("theta2"), 45:IVar("theta1"), 46:INeg(InstructionI(44)), 47:IAdd(InstructionI(45), I(InstructionI(46))), 48:IFuncSin(InstructionI(47)), 49:IExp { base: I(InstructionI(48)), power: C(2.0) }, 50:IAdd(InstructionI(49), C(1.0)), 51:IAdd(InstructionI(42), I(InstructionI(43))), 52:IInv(InstructionI(50)), 53:IVar("omega1"), 54:IVar("theta2"), 55:IVar("theta1"), 56:INeg(InstructionI(54)), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IExp { base: I(InstructionI(53)), power: C(2.0) }, 59:IFuncSin(InstructionI(57)), 60:IMul(InstructionI(58), I(InstructionI(59))), 61:IVar("omega2"), 62:IVar("omega2"), 63:IExp { base: I(InstructionI(62)), power: C(2.0) }, 64:IAdd(InstructionI(63), C(1e-10)), 65:IExp { base: I(InstructionI(64)), power: C(0.5) }, 66:IExp { base: I(InstructionI(61)), power: C(3.0) }, 67:IInv(InstructionI(65)), 68:IMul(InstructionI(66), I(InstructionI(67))), 69:IVar("omega2"), 70:IVar("theta2"), 71:IVar("theta1"), 72:INeg(InstructionI(70)), 73:IAdd(InstructionI(71), I(InstructionI(72))), 74:IVar("theta2"), 75:IVar("theta1"), 76:INeg(InstructionI(74)), 77:IAdd(InstructionI(75), I(InstructionI(76))), 78:IExp { base: I(InstructionI(69)), power: C(2.0) }, 79:IFuncSin(InstructionI(73)), 80:IMul(InstructionI(78), I(InstructionI(79))), 81:IFuncCos(InstructionI(77)), 82:IVar("theta1"), 83:IVar("theta2"), 84:IVar("theta1"), 85:INeg(InstructionI(83)), 86:IAdd(InstructionI(84), I(InstructionI(85))), 87:IFuncSin(InstructionI(82)), 88:IFuncCos(InstructionI(86)), 89:IMul(InstructionI(87), I(InstructionI(88))), 90:IVar("theta2"), 91:IFuncSin(InstructionI(90)), 92:IMul(InstructionI(91), C(19.62)), 93:INeg(InstructionI(68)), 94:IMul(InstructionI(60), C(2.0)), 95:IAdd(InstructionI(93), I(InstructionI(94))), 96:IMul(InstructionI(80), I(InstructionI(81))), 97:IAdd(InstructionI(95), I(InstructionI(96))), 98:IVar("u2"), 99:IAdd(InstructionI(97), I(InstructionI(98))), 100:INeg(InstructionI(92)), 101:IAdd(InstructionI(99), I(InstructionI(100))), 102:IMul(InstructionI(89), C(19.62)), 103:IVar("theta2"), 104:IVar("theta1"), 105:INeg(InstructionI(103)), 106:IAdd(InstructionI(104), I(InstructionI(105))), 107:IVar("theta2"), 108:IVar("theta1"), 109:INeg(InstructionI(107)), 110:IAdd(InstructionI(108), I(InstructionI(109))), 111:IVar("theta2"), 112:IVar("theta1"), 113:INeg(InstructionI(111)), 114:IAdd(InstructionI(112), I(InstructionI(113))), 115:IFuncSin(InstructionI(114)), 116:IExp { base: I(InstructionI(115)), power: C(2.0) }, 117:IAdd(InstructionI(116), C(1.0)), 118:IExp { base: I(InstructionI(117)), power: C(2.0) }, 119:IAdd(InstructionI(101), I(InstructionI(102))), 120:IFuncSin(InstructionI(106)), 121:IMul(InstructionI(119), I(InstructionI(120))), 122:IInv(InstructionI(118)), 123:IMul(InstructionI(121), I(InstructionI(122))), 124:IFuncCos(InstructionI(110)), 125:IMul(InstructionI(123), I(InstructionI(124))), 126:IMul(InstructionI(51), I(InstructionI(52))), 127:IMul(InstructionI(125), C(2.0)) } }), (IMul(InstructionI(40), I(InstructionI(41))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("omega2")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 1:Expression { first: EStdFunc(EVar("omega2")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 2:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 4:Expression { first: EConstant(1.0), pairs: [ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(4.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(0)))), ExprPair(EExp, EConstant(1.5)), ExprPair(ESub, EConstant(3.0)), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(1)))), ExprPair(EExp, EConstant(0.5)), ExprPair(EAdd, EConstant(2.0)), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(2)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(3))))] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 6:Expression { first: EStdFunc(EFuncSin(ExpressionI(5))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 7:Expression { first: EUnaryOp(EParentheses(ExpressionI(4))), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(6))))] } }, vals:{}, instrs:{ 0:IVar("omega2"), 1:IVar("omega2"), 2:IExp { base: I(InstructionI(1)), power: C(2.0) }, 3:IAdd(InstructionI(2), C(1e-10)), 4:IExp { base: I(InstructionI(3)), power: C(1.5) }, 5:IInv(InstructionI(4)), 6:IExp { base: I(InstructionI(0)), power: C(4.0) }, 7:IVar("omega2"), 8:IVar("omega2"), 9:IExp { base: I(InstructionI(8)), power: C(2.0) }, 10:IAdd(InstructionI(9), C(1e-10)), 11:IExp { base: I(InstructionI(10)), power: C(0.5) }, 12:IInv(InstructionI(11)), 13:IExp { base: I(InstructionI(7)), power: C(2.0) }, 14:IMul(InstructionI(12), I(InstructionI(13))), 15:IMul(InstructionI(14), C(3.0)), 16:IVar("theta2"), 17:IVar("theta1"), 18:INeg(InstructionI(16)), 19:IAdd(InstructionI(17), I(InstructionI(18))), 20:IVar("theta2"), 21:IVar("theta1"), 22:INeg(InstructionI(20)), 23:IAdd(InstructionI(21), I(InstructionI(22))), 24:IVar("omega2"), 25:IFuncSin(InstructionI(19)), 26:IMul(InstructionI(24), I(InstructionI(25))), 27:IFuncCos(InstructionI(23)), 28:IMul(InstructionI(26), I(InstructionI(27))), 29:INeg(InstructionI(15)), 30:IMul(InstructionI(5), I(InstructionI(6))), 31:IAdd(InstructionI(29), I(InstructionI(30))), 32:IMul(InstructionI(28), C(2.0)), 33:IVar("theta2"), 34:IVar("theta1"), 35:INeg(InstructionI(33)), 36:IAdd(InstructionI(34), I(InstructionI(35))), 37:IFuncSin(InstructionI(36)), 38:IExp { base: I(InstructionI(37)), power: C(2.0) }, 39:IAdd(InstructionI(38), C(1.0)), 40:IAdd(InstructionI(31), I(InstructionI(32))), 41:IInv(InstructionI(39)) } })]]*/

fn eval_dfdx(params: &[f64]) -> DMatrix<f64> {
    let mut v: Vec<Vec<f64>> = vec![vec![0.0; 4]; 4];
    let t0 = 0.00000000000000000e0;
    v[0][0] = t0;
    let t1 = 1.00000000000000000e0;
    v[0][1] = t1;
    let t2 = 0.00000000000000000e0;
    v[0][2] = t2;
    let t3 = 0.00000000000000000e0;
    v[0][3] = t3;
    let t4 = params[1];
    let t5 = params[2];
    let t6 = params[0];
    let t7 = -t5;
    let t8 = t6 + t7;
    let t9 = t8.sin();
    let t10 = t4.powf(2.00000000000000000e0);
    let t11 = t9.powf(2.00000000000000000e0);
    let t12 = params[1];
    let t13 = params[2];
    let t14 = params[0];
    let t15 = -t13;
    let t16 = t14 + t15;
    let t17 = t12.powf(2.00000000000000000e0);
    let t18 = t16.cos();
    let t19 = params[3];
    let t20 = t17 * t18;
    let t21 = t19.powf(2.00000000000000000e0);
    let t22 = params[2];
    let t23 = params[0];
    let t24 = -t22;
    let t25 = t23 + t24;
    let t26 = t20 + t21;
    let t27 = t25.cos();
    let t28 = t26 * t27;
    let t29 = params[2];
    let t30 = params[2];
    let t31 = params[0];
    let t32 = -t30;
    let t33 = t31 + t32;
    let t34 = t29.sin();
    let t35 = t33.sin();
    let t36 = t34 * t35;
    let t37 = t36 * 9.81000000000000050e0;
    let t38 = params[0];
    let t39 = t38.cos();
    let t40 = t39 * 1.96200000000000010e1;
    let t41 = t10 * t11;
    let t42 = -t28;
    let t43 = t41 + t42;
    let t44 = -t37;
    let t45 = t43 + t44;
    let t46 = -t40;
    let t47 = params[2];
    let t48 = params[0];
    let t49 = -t47;
    let t50 = t48 + t49;
    let t51 = t50.sin();
    let t52 = t51.powf(2.00000000000000000e0);
    let t53 = t52 + 1.00000000000000000e0;
    let t54 = t45 + t46;
    let t55 = 1.0 / t53;
    let t56 = params[1];
    let t57 = -t56;
    let t58 = params[1];
    let t59 = t58.powf(2.00000000000000000e0);
    let t60 = t59 + 1.00000000000000004e-10;
    let t61 = t60.powf(5.00000000000000000e-1);
    let t62 = t57.powf(3.00000000000000000e0);
    let t63 = 1.0 / t61;
    let t64 = params[1];
    let t65 = params[2];
    let t66 = params[0];
    let t67 = -t65;
    let t68 = t66 + t67;
    let t69 = t64.powf(2.00000000000000000e0);
    let t70 = t68.cos();
    let t71 = params[3];
    let t72 = t69 * t70;
    let t73 = t71.powf(2.00000000000000000e0);
    let t74 = params[2];
    let t75 = params[0];
    let t76 = -t74;
    let t77 = t75 + t76;
    let t78 = t72 + t73;
    let t79 = t77.sin();
    let t80 = t78 * t79;
    let t81 = params[0];
    let t82 = t81.sin();
    let t83 = t82 * 1.96200000000000010e1;
    let t84 = params[2];
    let t85 = params[2];
    let t86 = params[0];
    let t87 = -t85;
    let t88 = t86 + t87;
    let t89 = t84.sin();
    let t90 = t88.cos();
    let t91 = t89 * t90;
    let t92 = t62 * t63;
    let t93 = -t83;
    let t94 = t92 + t93;
    let t95 = -t80;
    let t96 = t94 + t95;
    let t97 = params[4];
    let t98 = t96 + t97;
    let t99 = t91 * 9.81000000000000050e0;
    let t100 = params[2];
    let t101 = params[0];
    let t102 = -t100;
    let t103 = t101 + t102;
    let t104 = params[2];
    let t105 = params[0];
    let t106 = -t104;
    let t107 = t105 + t106;
    let t108 = params[2];
    let t109 = params[0];
    let t110 = -t108;
    let t111 = t109 + t110;
    let t112 = t111.sin();
    let t113 = t112.powf(2.00000000000000000e0);
    let t114 = t113 + 1.00000000000000000e0;
    let t115 = t114.powf(2.00000000000000000e0);
    let t116 = t98 + t99;
    let t117 = t103.sin();
    let t118 = t116 * t117;
    let t119 = 1.0 / t115;
    let t120 = t118 * t119;
    let t121 = t107.cos();
    let t122 = t120 * t121;
    let t123 = t122 * 2.00000000000000000e0;
    let t124 = t54 * t55;
    let t125 = -t123;
    let t126 = t124 + t125;
    v[1][0] = t126;
    let t127 = params[1];
    let t128 = params[1];
    let t129 = t128.powf(2.00000000000000000e0);
    let t130 = t129 + 1.00000000000000004e-10;
    let t131 = t130.powf(1.50000000000000000e0);
    let t132 = 1.0 / t131;
    let t133 = t127.powf(4.00000000000000000e0);
    let t134 = params[1];
    let t135 = params[1];
    let t136 = t135.powf(2.00000000000000000e0);
    let t137 = t136 + 1.00000000000000004e-10;
    let t138 = t137.powf(5.00000000000000000e-1);
    let t139 = 1.0 / t138;
    let t140 = t134.powf(2.00000000000000000e0);
    let t141 = t139 * t140;
    let t142 = t141 * 3.00000000000000000e0;
    let t143 = params[2];
    let t144 = params[0];
    let t145 = -t143;
    let t146 = t144 + t145;
    let t147 = params[2];
    let t148 = params[0];
    let t149 = -t147;
    let t150 = t148 + t149;
    let t151 = params[1];
    let t152 = t146.sin();
    let t153 = t151 * t152;
    let t154 = t150.cos();
    let t155 = t153 * t154;
    let t156 = t155 * 2.00000000000000000e0;
    let t157 = t132 * t133;
    let t158 = -t142;
    let t159 = t157 + t158;
    let t160 = -t156;
    let t161 = params[2];
    let t162 = params[0];
    let t163 = -t161;
    let t164 = t162 + t163;
    let t165 = t164.sin();
    let t166 = t165.powf(2.00000000000000000e0);
    let t167 = t166 + 1.00000000000000000e0;
    let t168 = t159 + t160;
    let t169 = 1.0 / t167;
    let t170 = t168 * t169;
    v[1][1] = t170;
    let t171 = params[1];
    let t172 = -t171;
    let t173 = params[2];
    let t174 = params[0];
    let t175 = -t173;
    let t176 = t174 + t175;
    let t177 = t176.sin();
    let t178 = t172.powf(2.00000000000000000e0);
    let t179 = t177.powf(2.00000000000000000e0);
    let t180 = params[1];
    let t181 = params[2];
    let t182 = params[0];
    let t183 = -t181;
    let t184 = t182 + t183;
    let t185 = t180.powf(2.00000000000000000e0);
    let t186 = t184.cos();
    let t187 = params[3];
    let t188 = t185 * t186;
    let t189 = t187.powf(2.00000000000000000e0);
    let t190 = params[2];
    let t191 = params[0];
    let t192 = -t190;
    let t193 = t191 + t192;
    let t194 = t188 + t189;
    let t195 = t193.cos();
    let t196 = params[2];
    let t197 = params[2];
    let t198 = params[0];
    let t199 = -t197;
    let t200 = t198 + t199;
    let t201 = t196.sin();
    let t202 = t200.sin();
    let t203 = t201 * t202;
    let t204 = params[2];
    let t205 = params[2];
    let t206 = params[0];
    let t207 = -t205;
    let t208 = t206 + t207;
    let t209 = t204.cos();
    let t210 = t208.cos();
    let t211 = t209 * t210;
    let t212 = t178 * t179;
    let t213 = t194 * t195;
    let t214 = t212 + t213;
    let t215 = t203 * 9.81000000000000050e0;
    let t216 = t214 + t215;
    let t217 = t211 * 9.81000000000000050e0;
    let t218 = params[2];
    let t219 = params[0];
    let t220 = -t218;
    let t221 = t219 + t220;
    let t222 = t221.sin();
    let t223 = t222.powf(2.00000000000000000e0);
    let t224 = t223 + 1.00000000000000000e0;
    let t225 = t216 + t217;
    let t226 = 1.0 / t224;
    let t227 = params[1];
    let t228 = -t227;
    let t229 = params[1];
    let t230 = t229.powf(2.00000000000000000e0);
    let t231 = t230 + 1.00000000000000004e-10;
    let t232 = t231.powf(5.00000000000000000e-1);
    let t233 = t228.powf(3.00000000000000000e0);
    let t234 = 1.0 / t232;
    let t235 = params[1];
    let t236 = params[2];
    let t237 = params[0];
    let t238 = -t236;
    let t239 = t237 + t238;
    let t240 = t235.powf(2.00000000000000000e0);
    let t241 = t239.cos();
    let t242 = params[3];
    let t243 = t240 * t241;
    let t244 = t242.powf(2.00000000000000000e0);
    let t245 = params[2];
    let t246 = params[0];
    let t247 = -t245;
    let t248 = t246 + t247;
    let t249 = t243 + t244;
    let t250 = t248.sin();
    let t251 = t249 * t250;
    let t252 = params[0];
    let t253 = t252.sin();
    let t254 = t253 * 1.96200000000000010e1;
    let t255 = params[2];
    let t256 = params[2];
    let t257 = params[0];
    let t258 = -t256;
    let t259 = t257 + t258;
    let t260 = t255.sin();
    let t261 = t259.cos();
    let t262 = t260 * t261;
    let t263 = t233 * t234;
    let t264 = -t254;
    let t265 = t263 + t264;
    let t266 = -t251;
    let t267 = t265 + t266;
    let t268 = params[4];
    let t269 = t267 + t268;
    let t270 = t262 * 9.81000000000000050e0;
    let t271 = params[2];
    let t272 = params[0];
    let t273 = -t271;
    let t274 = t272 + t273;
    let t275 = params[2];
    let t276 = params[0];
    let t277 = -t275;
    let t278 = t276 + t277;
    let t279 = params[2];
    let t280 = params[0];
    let t281 = -t279;
    let t282 = t280 + t281;
    let t283 = t282.sin();
    let t284 = t283.powf(2.00000000000000000e0);
    let t285 = t284 + 1.00000000000000000e0;
    let t286 = t285.powf(2.00000000000000000e0);
    let t287 = t269 + t270;
    let t288 = t274.sin();
    let t289 = t287 * t288;
    let t290 = 1.0 / t286;
    let t291 = t289 * t290;
    let t292 = t278.cos();
    let t293 = t291 * t292;
    let t294 = t225 * t226;
    let t295 = t293 * 2.00000000000000000e0;
    let t296 = t294 + t295;
    v[1][2] = t296;
    let t297 = params[2];
    let t298 = params[0];
    let t299 = -t297;
    let t300 = t298 + t299;
    let t301 = params[2];
    let t302 = params[0];
    let t303 = -t301;
    let t304 = t302 + t303;
    let t305 = t304.sin();
    let t306 = t305.powf(2.00000000000000000e0);
    let t307 = t306 + 1.00000000000000000e0;
    let t308 = params[3];
    let t309 = 1.0 / t307;
    let t310 = t308 * t309;
    let t311 = t300.sin();
    let t312 = t310 * t311;
    let t313 = t312 * -2.00000000000000000e0;
    v[1][3] = t313;
    let t314 = 0.00000000000000000e0;
    v[2][0] = t314;
    let t315 = 0.00000000000000000e0;
    v[2][1] = t315;
    let t316 = 0.00000000000000000e0;
    v[2][2] = t316;
    let t317 = 1.00000000000000000e0;
    v[2][3] = t317;
    let t318 = params[1];
    let t319 = params[2];
    let t320 = params[0];
    let t321 = -t319;
    let t322 = t320 + t321;
    let t323 = t318.powf(2.00000000000000000e0);
    let t324 = t322.cos();
    let t325 = t323 * t324;
    let t326 = params[3];
    let t327 = params[2];
    let t328 = params[0];
    let t329 = -t327;
    let t330 = t328 + t329;
    let t331 = t330.sin();
    let t332 = t326.powf(2.00000000000000000e0);
    let t333 = t331.powf(2.00000000000000000e0);
    let t334 = t332 * t333;
    let t335 = params[3];
    let t336 = params[2];
    let t337 = params[0];
    let t338 = -t336;
    let t339 = t337 + t338;
    let t340 = t339.cos();
    let t341 = t335.powf(2.00000000000000000e0);
    let t342 = t340.powf(2.00000000000000000e0);
    let t343 = params[0];
    let t344 = params[2];
    let t345 = params[0];
    let t346 = -t344;
    let t347 = t345 + t346;
    let t348 = t343.sin();
    let t349 = t347.sin();
    let t350 = t348 * t349;
    let t351 = t350 * 1.96200000000000010e1;
    let t352 = params[0];
    let t353 = params[2];
    let t354 = params[0];
    let t355 = -t353;
    let t356 = t354 + t355;
    let t357 = t352.cos();
    let t358 = t356.cos();
    let t359 = t357 * t358;
    let t360 = -t334;
    let t361 = t325 * 2.00000000000000000e0;
    let t362 = t360 + t361;
    let t363 = -t351;
    let t364 = t362 + t363;
    let t365 = t341 * t342;
    let t366 = t364 + t365;
    let t367 = t359 * 1.96200000000000010e1;
    let t368 = params[2];
    let t369 = params[0];
    let t370 = -t368;
    let t371 = t369 + t370;
    let t372 = t371.sin();
    let t373 = t372.powf(2.00000000000000000e0);
    let t374 = t373 + 1.00000000000000000e0;
    let t375 = t366 + t367;
    let t376 = 1.0 / t374;
    let t377 = params[1];
    let t378 = params[2];
    let t379 = params[0];
    let t380 = -t378;
    let t381 = t379 + t380;
    let t382 = t377.powf(2.00000000000000000e0);
    let t383 = t381.sin();
    let t384 = t382 * t383;
    let t385 = params[3];
    let t386 = params[3];
    let t387 = t386.powf(2.00000000000000000e0);
    let t388 = t387 + 1.00000000000000004e-10;
    let t389 = t388.powf(5.00000000000000000e-1);
    let t390 = t385.powf(3.00000000000000000e0);
    let t391 = 1.0 / t389;
    let t392 = t390 * t391;
    let t393 = params[3];
    let t394 = params[2];
    let t395 = params[0];
    let t396 = -t394;
    let t397 = t395 + t396;
    let t398 = params[2];
    let t399 = params[0];
    let t400 = -t398;
    let t401 = t399 + t400;
    let t402 = t393.powf(2.00000000000000000e0);
    let t403 = t397.sin();
    let t404 = t402 * t403;
    let t405 = t401.cos();
    let t406 = params[0];
    let t407 = params[2];
    let t408 = params[0];
    let t409 = -t407;
    let t410 = t408 + t409;
    let t411 = t406.sin();
    let t412 = t410.cos();
    let t413 = t411 * t412;
    let t414 = params[2];
    let t415 = t414.sin();
    let t416 = t415 * 1.96200000000000010e1;
    let t417 = -t392;
    let t418 = t384 * 2.00000000000000000e0;
    let t419 = t417 + t418;
    let t420 = t404 * t405;
    let t421 = t419 + t420;
    let t422 = params[5];
    let t423 = t421 + t422;
    let t424 = -t416;
    let t425 = t423 + t424;
    let t426 = t413 * 1.96200000000000010e1;
    let t427 = params[2];
    let t428 = params[0];
    let t429 = -t427;
    let t430 = t428 + t429;
    let t431 = params[2];
    let t432 = params[0];
    let t433 = -t431;
    let t434 = t432 + t433;
    let t435 = params[2];
    let t436 = params[0];
    let t437 = -t435;
    let t438 = t436 + t437;
    let t439 = t438.sin();
    let t440 = t439.powf(2.00000000000000000e0);
    let t441 = t440 + 1.00000000000000000e0;
    let t442 = t441.powf(2.00000000000000000e0);
    let t443 = t425 + t426;
    let t444 = t430.sin();
    let t445 = t443 * t444;
    let t446 = 1.0 / t442;
    let t447 = t445 * t446;
    let t448 = t434.cos();
    let t449 = t447 * t448;
    let t450 = t449 * 2.00000000000000000e0;
    let t451 = t375 * t376;
    let t452 = -t450;
    let t453 = t451 + t452;
    v[3][0] = t453;
    let t454 = params[2];
    let t455 = params[0];
    let t456 = -t454;
    let t457 = t455 + t456;
    let t458 = params[2];
    let t459 = params[0];
    let t460 = -t458;
    let t461 = t459 + t460;
    let t462 = t461.sin();
    let t463 = t462.powf(2.00000000000000000e0);
    let t464 = t463 + 1.00000000000000000e0;
    let t465 = params[1];
    let t466 = 1.0 / t464;
    let t467 = t465 * t466;
    let t468 = t457.sin();
    let t469 = t467 * t468;
    let t470 = t469 * 4.00000000000000000e0;
    v[3][1] = t470;
    let t471 = params[1];
    let t472 = params[2];
    let t473 = params[0];
    let t474 = -t472;
    let t475 = t473 + t474;
    let t476 = t471.powf(2.00000000000000000e0);
    let t477 = t475.cos();
    let t478 = t476 * t477;
    let t479 = params[3];
    let t480 = params[2];
    let t481 = params[0];
    let t482 = -t480;
    let t483 = t481 + t482;
    let t484 = t483.sin();
    let t485 = t479.powf(2.00000000000000000e0);
    let t486 = t484.powf(2.00000000000000000e0);
    let t487 = params[3];
    let t488 = params[2];
    let t489 = params[0];
    let t490 = -t488;
    let t491 = t489 + t490;
    let t492 = t491.cos();
    let t493 = t487.powf(2.00000000000000000e0);
    let t494 = t492.powf(2.00000000000000000e0);
    let t495 = t493 * t494;
    let t496 = params[0];
    let t497 = params[2];
    let t498 = params[0];
    let t499 = -t497;
    let t500 = t498 + t499;
    let t501 = t496.sin();
    let t502 = t500.sin();
    let t503 = t501 * t502;
    let t504 = params[2];
    let t505 = t504.cos();
    let t506 = t505 * 1.96200000000000010e1;
    let t507 = t478 * -2.00000000000000000e0;
    let t508 = -t495;
    let t509 = t507 + t508;
    let t510 = t485 * t486;
    let t511 = t509 + t510;
    let t512 = -t506;
    let t513 = t511 + t512;
    let t514 = t503 * 1.96200000000000010e1;
    let t515 = params[2];
    let t516 = params[0];
    let t517 = -t515;
    let t518 = t516 + t517;
    let t519 = t518.sin();
    let t520 = t519.powf(2.00000000000000000e0);
    let t521 = t520 + 1.00000000000000000e0;
    let t522 = t513 + t514;
    let t523 = 1.0 / t521;
    let t524 = params[1];
    let t525 = params[2];
    let t526 = params[0];
    let t527 = -t525;
    let t528 = t526 + t527;
    let t529 = t524.powf(2.00000000000000000e0);
    let t530 = t528.sin();
    let t531 = t529 * t530;
    let t532 = params[3];
    let t533 = params[3];
    let t534 = t533.powf(2.00000000000000000e0);
    let t535 = t534 + 1.00000000000000004e-10;
    let t536 = t535.powf(5.00000000000000000e-1);
    let t537 = t532.powf(3.00000000000000000e0);
    let t538 = 1.0 / t536;
    let t539 = t537 * t538;
    let t540 = params[3];
    let t541 = params[2];
    let t542 = params[0];
    let t543 = -t541;
    let t544 = t542 + t543;
    let t545 = params[2];
    let t546 = params[0];
    let t547 = -t545;
    let t548 = t546 + t547;
    let t549 = t540.powf(2.00000000000000000e0);
    let t550 = t544.sin();
    let t551 = t549 * t550;
    let t552 = t548.cos();
    let t553 = params[0];
    let t554 = params[2];
    let t555 = params[0];
    let t556 = -t554;
    let t557 = t555 + t556;
    let t558 = t553.sin();
    let t559 = t557.cos();
    let t560 = t558 * t559;
    let t561 = params[2];
    let t562 = t561.sin();
    let t563 = t562 * 1.96200000000000010e1;
    let t564 = -t539;
    let t565 = t531 * 2.00000000000000000e0;
    let t566 = t564 + t565;
    let t567 = t551 * t552;
    let t568 = t566 + t567;
    let t569 = params[5];
    let t570 = t568 + t569;
    let t571 = -t563;
    let t572 = t570 + t571;
    let t573 = t560 * 1.96200000000000010e1;
    let t574 = params[2];
    let t575 = params[0];
    let t576 = -t574;
    let t577 = t575 + t576;
    let t578 = params[2];
    let t579 = params[0];
    let t580 = -t578;
    let t581 = t579 + t580;
    let t582 = params[2];
    let t583 = params[0];
    let t584 = -t582;
    let t585 = t583 + t584;
    let t586 = t585.sin();
    let t587 = t586.powf(2.00000000000000000e0);
    let t588 = t587 + 1.00000000000000000e0;
    let t589 = t588.powf(2.00000000000000000e0);
    let t590 = t572 + t573;
    let t591 = t577.sin();
    let t592 = t590 * t591;
    let t593 = 1.0 / t589;
    let t594 = t592 * t593;
    let t595 = t581.cos();
    let t596 = t594 * t595;
    let t597 = t522 * t523;
    let t598 = t596 * 2.00000000000000000e0;
    let t599 = t597 + t598;
    v[3][2] = t599;
    let t600 = params[3];
    let t601 = params[3];
    let t602 = t601.powf(2.00000000000000000e0);
    let t603 = t602 + 1.00000000000000004e-10;
    let t604 = t603.powf(1.50000000000000000e0);
    let t605 = 1.0 / t604;
    let t606 = t600.powf(4.00000000000000000e0);
    let t607 = params[3];
    let t608 = params[3];
    let t609 = t608.powf(2.00000000000000000e0);
    let t610 = t609 + 1.00000000000000004e-10;
    let t611 = t610.powf(5.00000000000000000e-1);
    let t612 = 1.0 / t611;
    let t613 = t607.powf(2.00000000000000000e0);
    let t614 = t612 * t613;
    let t615 = t614 * 3.00000000000000000e0;
    let t616 = params[2];
    let t617 = params[0];
    let t618 = -t616;
    let t619 = t617 + t618;
    let t620 = params[2];
    let t621 = params[0];
    let t622 = -t620;
    let t623 = t621 + t622;
    let t624 = params[3];
    let t625 = t619.sin();
    let t626 = t624 * t625;
    let t627 = t623.cos();
    let t628 = t626 * t627;
    let t629 = -t615;
    let t630 = t605 * t606;
    let t631 = t629 + t630;
    let t632 = t628 * 2.00000000000000000e0;
    let t633 = params[2];
    let t634 = params[0];
    let t635 = -t633;
    let t636 = t634 + t635;
    let t637 = t636.sin();
    let t638 = t637.powf(2.00000000000000000e0);
    let t639 = t638 + 1.00000000000000000e0;
    let t640 = t631 + t632;
    let t641 = 1.0 / t639;
    let t642 = t640 * t641;
    v[3][3] = t642;
    vec_to_dmat(&v)
}
/*
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IFuncSin(InstructionI(3)), 5:IExp { base: I(InstructionI(4)), power: C(2.0) }, 6:IAdd(InstructionI(5), C(1.0)) } }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IFuncSin(InstructionI(3)), 5:IExp { base: I(InstructionI(4)), power: C(2.0) }, 6:IAdd(InstructionI(5), C(1.0)) } }



  [[(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} })], [(IInv(InstructionI(6)), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EFuncSin(ExpressionI(0))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 2:Expression { first: EConstant(1.0), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(1))))] } }, vals:{}, instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IFuncSin(InstructionI(3)), 5:IExp { base: I(InstructionI(4)), power: C(2.0) }, 6:IAdd(InstructionI(5), C(1.0)) } }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} })], [(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} })], [(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IInv(InstructionI(6)), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EFuncSin(ExpressionI(0))), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1.0))] }, 2:Expression { first: EConstant(1.0), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(1))))] } }, vals:{}, instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IFuncSin(InstructionI(3)), 5:IExp { base: I(InstructionI(4)), power: C(2.0) }, 6:IAdd(InstructionI(5), C(1.0)) } })]]*/

fn eval_dfdu(params: &[f64]) -> DMatrix<f64> {
    let mut v: Vec<Vec<f64>> = vec![vec![0.0; 2]; 4];
    let t0 = 0.00000000000000000e0;
    v[0][0] = t0;
    let t1 = 0.00000000000000000e0;
    v[0][1] = t1;
    let t2 = params[2];
    let t3 = params[0];
    let t4 = -t2;
    let t5 = t3 + t4;
    let t6 = t5.sin();
    let t7 = t6.powf(2.00000000000000000e0);
    let t8 = t7 + 1.00000000000000000e0;
    let t9 = 1.0 / t8;
    v[1][0] = t9;
    let t10 = 0.00000000000000000e0;
    v[1][1] = t10;
    let t11 = 0.00000000000000000e0;
    v[2][0] = t11;
    let t12 = 0.00000000000000000e0;
    v[2][1] = t12;
    let t13 = 0.00000000000000000e0;
    v[3][0] = t13;
    let t14 = params[2];
    let t15 = params[0];
    let t16 = -t14;
    let t17 = t15 + t16;
    let t18 = t17.sin();
    let t19 = t18.powf(2.00000000000000000e0);
    let t20 = t19 + 1.00000000000000000e0;
    let t21 = 1.0 / t20;
    v[3][1] = t21;
    vec_to_dmat(&v)
}

use std::f64::consts::PI;
use std::sync::Arc;

use control_rs::numeric_services::symbolic::{
    ExprRegistry, SymbolicExpr, SymbolicFunction, TryIntoEvalResult,
};
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::rk4::RK4Numeric;
use control_rs::physics::discretizer::{NumericDiscretizer, RK4Symbolic, SymbolicDiscretizer};
use control_rs::physics::models::{DoublePendulum, DoublePendulumState};
use control_rs::physics::traits::{Dynamics, State, SymbolicDynamics};
use control_rs::utils::evaluable::Evaluable;
use control_rs::utils::matrix::vec_to_dmat;
use nalgebra::{DMatrix, DVector};
use std::time::Instant;

fn main() {
    let m1 = 1.0;
    let m2 = 1.0;
    let l1 = 1.0;
    let l2 = 1.0;
    let air_resistance_coeff = 0.0;

    let theta1 = PI / 1.6;
    let omega1 = 0.0;
    let theta2 = PI / 1.8;
    let omega2 = 0.0;
    let dt = 0.01;

    let registry = Arc::new(ExprRegistry::new());

    let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(&registry));
    let state_0 = DoublePendulumState::new(theta1, omega1, theta2, omega2);
    let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
    let input_symbol = registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
    let jacobian_symbols = state_symbol.extend(&input_symbol);
    let symbolic_f = model.dynamics_symbolic(&state_symbol, &registry);

    let params_state = DVector::from_vec(vec![theta1, omega1, theta2, omega2]);
    let params_input = DVector::from_vec(vec![0.0, 0.0]);
    let mut params = params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());

    let symbolic_eval_f: DVector<f64> =
        SymbolicFunction::new(symbolic_f.to_fn(&registry).unwrap(), &jacobian_symbols)
            .eval(&params)
            .try_into_eval_result()
            .unwrap();

    let symbolic_f_dx = SymbolicFunction::new(
        symbolic_f
            .jacobian(&state_symbol)
            .unwrap()
            .to_fn(&registry)
            .unwrap(),
        &jacobian_symbols,
    )
    .evaluate(&params)
    .unwrap();
    let symbolic_f_du = SymbolicFunction::new(
        symbolic_f
            .jacobian(&input_symbol)
            .unwrap()
            .to_fn(&registry)
            .unwrap(),
        &jacobian_symbols,
    )
    .evaluate(&params)
    .unwrap();

    /////
    let discretizer = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);
    let fa = discretizer.jacobian_x().unwrap();
    let fb = discretizer.jacobian_u().unwrap();
    let start = Instant::now();
    let a = fa.evaluate(&params).unwrap();
    let b = fb.evaluate(&params).unwrap();
    let duration_symbolic = start.elapsed();

    //let c = discretizer.step(&model, &state_0, None, dt).unwrap();

    let start = Instant::now();

    let df_dx = eval_dfdx(&params);
    let df_du = eval_dfdu(&params);
    let f = model.dynamics(&state_0, None).to_vec();

    //RK4
    let k1 = DVector::from_vec(f.clone());
    let dk1_dx = &df_dx;
    let dk1_du = &df_du;

    let new_params_state = &params_state + dt / 2.0 * &k1;
    params = new_params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());
    let k2 = model
        .dynamics(
            &DoublePendulumState::from_slice(new_params_state.as_slice()),
            None,
        )
        .to_vector();
    let a_k2 = eval_dfdx(&params);
    let b_k2 = eval_dfdu(&params);
    let tmp_dx = &a_k2 * dk1_dx * dt / 2.0;
    let dk2_dx = &tmp_dx + &a_k2;
    let tmp_du = &a_k2 * dk1_du * dt / 2.0;
    let dk2_du = &tmp_du + b_k2;

    let new_params_state = &params_state + dt / 2.0 * &k2;
    params = new_params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());
    let k3 = model
        .dynamics(
            &DoublePendulumState::from_slice(new_params_state.as_slice()),
            None,
        )
        .to_vector();
    let a_k3 = eval_dfdx(&params);
    let b_k3 = eval_dfdu(&params);
    let tmp_dx = &a_k3 * &dk2_dx * dt / 2.0;
    let dk3_dx = &tmp_dx + &a_k3;
    let tmp_du = &a_k3 * &dk2_du * dt / 2.0;
    let dk3_du = &tmp_du + b_k3;

    let new_params_state = &params_state + dt * &k3;
    params = new_params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());
    //let k4 = DVector::from_vec(eval_f(&params));
    let a_k4 = eval_dfdx(&params);
    let b_k4 = eval_dfdu(&params);
    let tmp_dx = &a_k4 * &dk3_dx * dt;
    let dk4_dx = &tmp_dx + &a_k4;
    let tmp_du = &a_k4 * &dk3_du * dt;
    let dk4_du = &tmp_du + b_k4;

    let dx_next_dx =
        dt / 6.0 * (dk1_dx + 2.0 * dk2_dx + 2.0 * dk3_dx + dk4_dx) + DMatrix::identity(4, 4);
    let dx_next_du = dt / 6.0 * (dk1_du + 2.0 * dk2_du + 2.0 * dk3_du + dk4_du);
    let duration_numeric = start.elapsed();

    println!("{}, {:?}", symbolic_f_dx, df_dx);
    println!("{}, {:?}", symbolic_f_du, df_du);
    println!("{}, {:?}", symbolic_eval_f, f);
    println!("{}, {}", dx_next_dx, a);
    println!("{}, {}", dx_next_du, b);

    println!("{:?}, {:?}", duration_symbolic, duration_numeric);

    //
    let discretizer = RK4Numeric::new(&model, Box::new(eval_dfdx), Box::new(eval_dfdu)).unwrap();
    let (j1, j2) = discretizer.jacobians(&model, &state_0, None, dt);
    println!("{}, {}", j1, j2);
}
