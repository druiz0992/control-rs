/*
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{ 0:IVar("air_resistance_coeff"), 1:IVar("omega1"), 2:IVar("omega1"), 3:IExp { base: I(InstructionI(2)), power: C(2.0) }, 4:IAdd(InstructionI(3), C(1e-10)), 5:IExp { base: I(InstructionI(4)), power: C(0.5) }, 6:INeg(InstructionI(0)), 7:IInv(InstructionI(5)), 8:IMul(InstructionI(6), I(InstructionI(7))), 9:IExp { base: I(InstructionI(1)), power: C(3.0) }, 10:IVar("omega1"), 11:IVar("theta2"), 12:IVar("theta1"), 13:INeg(InstructionI(11)), 14:IAdd(InstructionI(12), I(InstructionI(13))), 15:IVar("l1"), 16:IExp { base: I(InstructionI(10)), power: C(2.0) }, 17:IMul(InstructionI(15), I(InstructionI(16))), 18:IFuncCos(InstructionI(14)), 19:IVar("omega2"), 20:IVar("l2"), 21:IExp { base: I(InstructionI(19)), power: C(2.0) }, 22:IMul(InstructionI(17), I(InstructionI(18))), 23:IMul(InstructionI(20), I(InstructionI(21))), 24:IVar("theta2"), 25:IVar("theta1"), 26:INeg(InstructionI(24)), 27:IAdd(InstructionI(25), I(InstructionI(26))), 28:IVar("m2"), 29:IAdd(InstructionI(22), I(InstructionI(23))), 30:IMul(InstructionI(28), I(InstructionI(29))), 31:IFuncSin(InstructionI(27)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IVar("theta2"), 34:IVar("theta2"), 35:IVar("theta1"), 36:INeg(InstructionI(34)), 37:IAdd(InstructionI(35), I(InstructionI(36))), 38:IVar("m2"), 39:IFuncSin(InstructionI(33)), 40:IMul(InstructionI(38), I(InstructionI(39))), 41:IFuncCos(InstructionI(37)), 42:IMul(InstructionI(40), I(InstructionI(41))), 43:IVar("m1"), 44:IVar("m2"), 45:IMul(InstructionI(43), C(9.81)), 46:IMul(InstructionI(44), C(9.81)), 47:IVar("theta1"), 48:IAdd(InstructionI(45), I(InstructionI(46))), 49:IFuncSin(InstructionI(47)), 50:IMul(InstructionI(48), I(InstructionI(49))), 51:INeg(InstructionI(32)), 52:IMul(InstructionI(8), I(InstructionI(9))), 53:IAdd(InstructionI(51), I(InstructionI(52))), 54:IMul(InstructionI(42), C(9.81)), 55:IAdd(InstructionI(53), I(InstructionI(54))), 56:INeg(InstructionI(50)), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IVar("u1"), 59:IVar("theta2"), 60:IVar("theta1"), 61:INeg(InstructionI(59)), 62:IAdd(InstructionI(60), I(InstructionI(61))), 63:IVar("theta2"), 64:IVar("theta1"), 65:INeg(InstructionI(63)), 66:IAdd(InstructionI(64), I(InstructionI(65))), 67:IVar("theta2"), 68:IVar("theta1"), 69:INeg(InstructionI(67)), 70:IAdd(InstructionI(68), I(InstructionI(69))), 71:IFuncSin(InstructionI(70)), 72:IVar("m2"), 73:IExp { base: I(InstructionI(71)), power: C(2.0) }, 74:IVar("m1"), 75:IMul(InstructionI(72), I(InstructionI(73))), 76:IAdd(InstructionI(74), I(InstructionI(75))), 77:IVar("l1"), 78:IExp { base: I(InstructionI(76)), power: C(2.0) }, 79:IMul(InstructionI(77), I(InstructionI(78))), 80:IVar("m2"), 81:IAdd(InstructionI(57), I(InstructionI(58))), 82:IMul(InstructionI(80), I(InstructionI(81))), 83:IFuncSin(InstructionI(62)), 84:IMul(InstructionI(82), I(InstructionI(83))), 85:IInv(InstructionI(79)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IFuncCos(InstructionI(66)), 88:IMul(InstructionI(86), I(InstructionI(87))), 89:IVar("omega1"), 90:IVar("theta2"), 91:IVar("theta1"), 92:INeg(InstructionI(90)), 93:IAdd(InstructionI(91), I(InstructionI(92))), 94:IFuncSin(InstructionI(93)), 95:IVar("l1"), 96:IVar("m2"), 97:IMul(InstructionI(95), I(InstructionI(96))), 98:IExp { base: I(InstructionI(89)), power: C(2.0) }, 99:IMul(InstructionI(97), I(InstructionI(98))), 100:IExp { base: I(InstructionI(94)), power: C(2.0) }, 101:IVar("omega1"), 102:IVar("theta2"), 103:IVar("theta1"), 104:INeg(InstructionI(102)), 105:IAdd(InstructionI(103), I(InstructionI(104))), 106:IVar("l1"), 107:IExp { base: I(InstructionI(101)), power: C(2.0) }, 108:IMul(InstructionI(106), I(InstructionI(107))), 109:IFuncCos(InstructionI(105)), 110:IVar("omega2"), 111:IVar("l2"), 112:IExp { base: I(InstructionI(110)), power: C(2.0) }, 113:IMul(InstructionI(108), I(InstructionI(109))), 114:IMul(InstructionI(111), I(InstructionI(112))), 115:IVar("theta2"), 116:IVar("theta1"), 117:INeg(InstructionI(115)), 118:IAdd(InstructionI(116), I(InstructionI(117))), 119:IVar("m2"), 120:IAdd(InstructionI(113), I(InstructionI(114))), 121:IMul(InstructionI(119), I(InstructionI(120))), 122:IFuncCos(InstructionI(118)), 123:IMul(InstructionI(121), I(InstructionI(122))), 124:IVar("theta2"), 125:IVar("theta2"), 126:IVar("theta1"), 127:INeg(InstructionI(125)), 128:IAdd(InstructionI(126), I(InstructionI(127))), 129:IVar("m2"), 130:IFuncSin(InstructionI(124)), 131:IMul(InstructionI(129), I(InstructionI(130))), 132:IFuncSin(InstructionI(128)), 133:IMul(InstructionI(131), I(InstructionI(132))), 134:IMul(InstructionI(133), C(9.81)), 135:IVar("m1"), 136:IVar("m2"), 137:IMul(InstructionI(135), C(9.81)), 138:IMul(InstructionI(136), C(9.81)), 139:IVar("theta1"), 140:IAdd(InstructionI(137), I(InstructionI(138))), 141:IFuncCos(InstructionI(139)), 142:IMul(InstructionI(140), I(InstructionI(141))), 143:IMul(InstructionI(99), I(InstructionI(100))), 144:INeg(InstructionI(123)), 145:IAdd(InstructionI(143), I(InstructionI(144))), 146:INeg(InstructionI(134)), 147:IAdd(InstructionI(145), I(InstructionI(146))), 148:INeg(InstructionI(142)), 149:IVar("theta2"), 150:IVar("theta1"), 151:INeg(InstructionI(149)), 152:IAdd(InstructionI(150), I(InstructionI(151))), 153:IFuncSin(InstructionI(152)), 154:IVar("m2"), 155:IExp { base: I(InstructionI(153)), power: C(2.0) }, 156:IVar("m1"), 157:IMul(InstructionI(154), I(InstructionI(155))), 158:IVar("l1"), 159:IAdd(InstructionI(156), I(InstructionI(157))), 160:IMul(InstructionI(158), I(InstructionI(159))), 161:IAdd(InstructionI(147), I(InstructionI(148))), 162:IInv(InstructionI(160)), 163:IMul(InstructionI(88), C(-2.0)), 164:IMul(InstructionI(161), I(InstructionI(162))) } }
CompileSlab{ instrs:{ 0:IVar("omega1"), 1:IVar("omega1"), 2:IExp { base: I(InstructionI(1)), power: C(2.0) }, 3:IAdd(InstructionI(2), C(1e-10)), 4:IExp { base: I(InstructionI(3)), power: C(1.5) }, 5:IVar("air_resistance_coeff"), 6:IInv(InstructionI(4)), 7:IMul(InstructionI(5), I(InstructionI(6))), 8:IExp { base: I(InstructionI(0)), power: C(4.0) }, 9:IVar("omega1"), 10:IVar("omega1"), 11:IExp { base: I(InstructionI(10)), power: C(2.0) }, 12:IAdd(InstructionI(11), C(1e-10)), 13:IExp { base: I(InstructionI(12)), power: C(0.5) }, 14:IVar("air_resistance_coeff"), 15:IInv(InstructionI(13)), 16:IMul(InstructionI(14), I(InstructionI(15))), 17:IExp { base: I(InstructionI(9)), power: C(2.0) }, 18:IMul(InstructionI(16), I(InstructionI(17))), 19:IMul(InstructionI(18), C(3.0)), 20:IVar("theta2"), 21:IVar("theta1"), 22:INeg(InstructionI(20)), 23:IAdd(InstructionI(21), I(InstructionI(22))), 24:IVar("theta2"), 25:IVar("theta1"), 26:INeg(InstructionI(24)), 27:IAdd(InstructionI(25), I(InstructionI(26))), 28:IVar("l1"), 29:IVar("m2"), 30:IMul(InstructionI(28), I(InstructionI(29))), 31:IVar("omega1"), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IFuncSin(InstructionI(23)), 34:IMul(InstructionI(32), I(InstructionI(33))), 35:IFuncCos(InstructionI(27)), 36:IMul(InstructionI(34), I(InstructionI(35))), 37:IMul(InstructionI(36), C(2.0)), 38:IMul(InstructionI(7), I(InstructionI(8))), 39:INeg(InstructionI(19)), 40:IAdd(InstructionI(38), I(InstructionI(39))), 41:INeg(InstructionI(37)), 42:IVar("theta2"), 43:IVar("theta1"), 44:INeg(InstructionI(42)), 45:IAdd(InstructionI(43), I(InstructionI(44))), 46:IFuncSin(InstructionI(45)), 47:IVar("m2"), 48:IExp { base: I(InstructionI(46)), power: C(2.0) }, 49:IVar("m1"), 50:IMul(InstructionI(47), I(InstructionI(48))), 51:IVar("l1"), 52:IAdd(InstructionI(49), I(InstructionI(50))), 53:IMul(InstructionI(51), I(InstructionI(52))), 54:IAdd(InstructionI(40), I(InstructionI(41))), 55:IInv(InstructionI(53)) } }
CompileSlab{ instrs:{ 0:IVar("air_resistance_coeff"), 1:IVar("omega1"), 2:IVar("omega1"), 3:IExp { base: I(InstructionI(2)), power: C(2.0) }, 4:IAdd(InstructionI(3), C(1e-10)), 5:IExp { base: I(InstructionI(4)), power: C(0.5) }, 6:INeg(InstructionI(0)), 7:IInv(InstructionI(5)), 8:IMul(InstructionI(6), I(InstructionI(7))), 9:IExp { base: I(InstructionI(1)), power: C(3.0) }, 10:IVar("omega1"), 11:IVar("theta2"), 12:IVar("theta1"), 13:INeg(InstructionI(11)), 14:IAdd(InstructionI(12), I(InstructionI(13))), 15:IVar("l1"), 16:IExp { base: I(InstructionI(10)), power: C(2.0) }, 17:IMul(InstructionI(15), I(InstructionI(16))), 18:IFuncCos(InstructionI(14)), 19:IVar("omega2"), 20:IVar("l2"), 21:IExp { base: I(InstructionI(19)), power: C(2.0) }, 22:IMul(InstructionI(17), I(InstructionI(18))), 23:IMul(InstructionI(20), I(InstructionI(21))), 24:IVar("theta2"), 25:IVar("theta1"), 26:INeg(InstructionI(24)), 27:IAdd(InstructionI(25), I(InstructionI(26))), 28:IVar("m2"), 29:IAdd(InstructionI(22), I(InstructionI(23))), 30:IMul(InstructionI(28), I(InstructionI(29))), 31:IFuncSin(InstructionI(27)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IVar("theta2"), 34:IVar("theta2"), 35:IVar("theta1"), 36:INeg(InstructionI(34)), 37:IAdd(InstructionI(35), I(InstructionI(36))), 38:IVar("m2"), 39:IFuncSin(InstructionI(33)), 40:IMul(InstructionI(38), I(InstructionI(39))), 41:IFuncCos(InstructionI(37)), 42:IMul(InstructionI(40), I(InstructionI(41))), 43:IVar("m1"), 44:IVar("m2"), 45:IMul(InstructionI(43), C(9.81)), 46:IMul(InstructionI(44), C(9.81)), 47:IVar("theta1"), 48:IAdd(InstructionI(45), I(InstructionI(46))), 49:IFuncSin(InstructionI(47)), 50:IMul(InstructionI(48), I(InstructionI(49))), 51:INeg(InstructionI(32)), 52:IMul(InstructionI(8), I(InstructionI(9))), 53:IAdd(InstructionI(51), I(InstructionI(52))), 54:IMul(InstructionI(42), C(9.81)), 55:IAdd(InstructionI(53), I(InstructionI(54))), 56:INeg(InstructionI(50)), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IVar("u1"), 59:IVar("theta2"), 60:IVar("theta1"), 61:INeg(InstructionI(59)), 62:IAdd(InstructionI(60), I(InstructionI(61))), 63:IVar("theta2"), 64:IVar("theta1"), 65:INeg(InstructionI(63)), 66:IAdd(InstructionI(64), I(InstructionI(65))), 67:IVar("theta2"), 68:IVar("theta1"), 69:INeg(InstructionI(67)), 70:IAdd(InstructionI(68), I(InstructionI(69))), 71:IFuncSin(InstructionI(70)), 72:IVar("m2"), 73:IExp { base: I(InstructionI(71)), power: C(2.0) }, 74:IVar("m1"), 75:IMul(InstructionI(72), I(InstructionI(73))), 76:IAdd(InstructionI(74), I(InstructionI(75))), 77:IVar("l1"), 78:IExp { base: I(InstructionI(76)), power: C(2.0) }, 79:IMul(InstructionI(77), I(InstructionI(78))), 80:IVar("m2"), 81:IAdd(InstructionI(57), I(InstructionI(58))), 82:IMul(InstructionI(80), I(InstructionI(81))), 83:IFuncSin(InstructionI(62)), 84:IMul(InstructionI(82), I(InstructionI(83))), 85:IInv(InstructionI(79)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IFuncCos(InstructionI(66)), 88:IMul(InstructionI(86), I(InstructionI(87))), 89:IVar("l1"), 90:IVar("omega1"), 91:IVar("theta2"), 92:IVar("theta1"), 93:INeg(InstructionI(91)), 94:IAdd(InstructionI(92), I(InstructionI(93))), 95:IFuncSin(InstructionI(94)), 96:INeg(InstructionI(89)), 97:IVar("m2"), 98:IMul(InstructionI(96), I(InstructionI(97))), 99:IExp { base: I(InstructionI(90)), power: C(2.0) }, 100:IMul(InstructionI(98), I(InstructionI(99))), 101:IExp { base: I(InstructionI(95)), power: C(2.0) }, 102:IVar("omega1"), 103:IVar("theta2"), 104:IVar("theta1"), 105:INeg(InstructionI(103)), 106:IAdd(InstructionI(104), I(InstructionI(105))), 107:IVar("l1"), 108:IExp { base: I(InstructionI(102)), power: C(2.0) }, 109:IMul(InstructionI(107), I(InstructionI(108))), 110:IFuncCos(InstructionI(106)), 111:IVar("omega2"), 112:IVar("l2"), 113:IExp { base: I(InstructionI(111)), power: C(2.0) }, 114:IMul(InstructionI(109), I(InstructionI(110))), 115:IMul(InstructionI(112), I(InstructionI(113))), 116:IVar("theta2"), 117:IVar("theta1"), 118:INeg(InstructionI(116)), 119:IAdd(InstructionI(117), I(InstructionI(118))), 120:IVar("m2"), 121:IAdd(InstructionI(114), I(InstructionI(115))), 122:IMul(InstructionI(120), I(InstructionI(121))), 123:IFuncCos(InstructionI(119)), 124:IVar("theta2"), 125:IVar("theta2"), 126:IVar("theta1"), 127:INeg(InstructionI(125)), 128:IAdd(InstructionI(126), I(InstructionI(127))), 129:IVar("m2"), 130:IFuncSin(InstructionI(124)), 131:IMul(InstructionI(129), I(InstructionI(130))), 132:IFuncSin(InstructionI(128)), 133:IMul(InstructionI(131), I(InstructionI(132))), 134:IVar("theta2"), 135:IVar("theta2"), 136:IVar("theta1"), 137:INeg(InstructionI(135)), 138:IAdd(InstructionI(136), I(InstructionI(137))), 139:IVar("m2"), 140:IFuncCos(InstructionI(134)), 141:IMul(InstructionI(139), I(InstructionI(140))), 142:IFuncCos(InstructionI(138)), 143:IMul(InstructionI(141), I(InstructionI(142))), 144:IMul(InstructionI(100), I(InstructionI(101))), 145:IMul(InstructionI(122), I(InstructionI(123))), 146:IAdd(InstructionI(144), I(InstructionI(145))), 147:IMul(InstructionI(133), C(9.81)), 148:IAdd(InstructionI(146), I(InstructionI(147))), 149:IMul(InstructionI(143), C(9.81)), 150:IVar("theta2"), 151:IVar("theta1"), 152:INeg(InstructionI(150)), 153:IAdd(InstructionI(151), I(InstructionI(152))), 154:IFuncSin(InstructionI(153)), 155:IVar("m2"), 156:IExp { base: I(InstructionI(154)), power: C(2.0) }, 157:IVar("m1"), 158:IMul(InstructionI(155), I(InstructionI(156))), 159:IVar("l1"), 160:IAdd(InstructionI(157), I(InstructionI(158))), 161:IMul(InstructionI(159), I(InstructionI(160))), 162:IAdd(InstructionI(148), I(InstructionI(149))), 163:IInv(InstructionI(161)), 164:IMul(InstructionI(88), C(2.0)), 165:IMul(InstructionI(162), I(InstructionI(163))) } }
CompileSlab{ instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IVar("theta2"), 5:IVar("theta1"), 6:INeg(InstructionI(4)), 7:IAdd(InstructionI(5), I(InstructionI(6))), 8:IFuncSin(InstructionI(7)), 9:IVar("m2"), 10:IExp { base: I(InstructionI(8)), power: C(2.0) }, 11:IVar("m1"), 12:IMul(InstructionI(9), I(InstructionI(10))), 13:IVar("l1"), 14:IAdd(InstructionI(11), I(InstructionI(12))), 15:IMul(InstructionI(13), I(InstructionI(14))), 16:IVar("l2"), 17:IVar("m2"), 18:IMul(InstructionI(16), I(InstructionI(17))), 19:IVar("omega2"), 20:IMul(InstructionI(18), I(InstructionI(19))), 21:IInv(InstructionI(15)), 22:IMul(InstructionI(20), I(InstructionI(21))), 23:IFuncSin(InstructionI(3)), 24:IMul(InstructionI(22), I(InstructionI(23))) } }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{ 0:IVar("air_resistance_coeff"), 1:IVar("omega2"), 2:IVar("omega2"), 3:IExp { base: I(InstructionI(2)), power: C(2.0) }, 4:IAdd(InstructionI(3), C(1e-10)), 5:IExp { base: I(InstructionI(4)), power: C(0.5) }, 6:INeg(InstructionI(0)), 7:IInv(InstructionI(5)), 8:IMul(InstructionI(6), I(InstructionI(7))), 9:IExp { base: I(InstructionI(1)), power: C(3.0) }, 10:IVar("omega2"), 11:IVar("theta2"), 12:IVar("theta1"), 13:INeg(InstructionI(11)), 14:IAdd(InstructionI(12), I(InstructionI(13))), 15:IVar("theta2"), 16:IVar("theta1"), 17:INeg(InstructionI(15)), 18:IAdd(InstructionI(16), I(InstructionI(17))), 19:IVar("l2"), 20:IVar("m2"), 21:IMul(InstructionI(19), I(InstructionI(20))), 22:IExp { base: I(InstructionI(10)), power: C(2.0) }, 23:IMul(InstructionI(21), I(InstructionI(22))), 24:IFuncSin(InstructionI(14)), 25:IMul(InstructionI(23), I(InstructionI(24))), 26:IFuncCos(InstructionI(18)), 27:IVar("m1"), 28:IVar("m2"), 29:IVar("omega1"), 30:IVar("theta2"), 31:IVar("theta1"), 32:INeg(InstructionI(30)), 33:IAdd(InstructionI(31), I(InstructionI(32))), 34:IVar("l1"), 35:IExp { base: I(InstructionI(29)), power: C(2.0) }, 36:IMul(InstructionI(34), I(InstructionI(35))), 37:IFuncSin(InstructionI(33)), 38:IVar("theta1"), 39:IVar("theta2"), 40:IVar("theta1"), 41:INeg(InstructionI(39)), 42:IAdd(InstructionI(40), I(InstructionI(41))), 43:IFuncSin(InstructionI(38)), 44:IFuncCos(InstructionI(42)), 45:IMul(InstructionI(43), I(InstructionI(44))), 46:IVar("theta2"), 47:IFuncSin(InstructionI(46)), 48:IMul(InstructionI(47), C(9.81)), 49:IMul(InstructionI(36), I(InstructionI(37))), 50:INeg(InstructionI(48)), 51:IAdd(InstructionI(49), I(InstructionI(50))), 52:IMul(InstructionI(45), C(9.81)), 53:IAdd(InstructionI(27), I(InstructionI(28))), 54:IAdd(InstructionI(51), I(InstructionI(52))), 55:IMul(InstructionI(8), I(InstructionI(9))), 56:IMul(InstructionI(25), I(InstructionI(26))), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IVar("u2"), 59:IAdd(InstructionI(57), I(InstructionI(58))), 60:IMul(InstructionI(53), I(InstructionI(54))), 61:IVar("theta2"), 62:IVar("theta1"), 63:INeg(InstructionI(61)), 64:IAdd(InstructionI(62), I(InstructionI(63))), 65:IVar("theta2"), 66:IVar("theta1"), 67:INeg(InstructionI(65)), 68:IAdd(InstructionI(66), I(InstructionI(67))), 69:IVar("theta2"), 70:IVar("theta1"), 71:INeg(InstructionI(69)), 72:IAdd(InstructionI(70), I(InstructionI(71))), 73:IFuncSin(InstructionI(72)), 74:IVar("m2"), 75:IExp { base: I(InstructionI(73)), power: C(2.0) }, 76:IVar("m1"), 77:IMul(InstructionI(74), I(InstructionI(75))), 78:IAdd(InstructionI(76), I(InstructionI(77))), 79:IVar("l2"), 80:IExp { base: I(InstructionI(78)), power: C(2.0) }, 81:IMul(InstructionI(79), I(InstructionI(80))), 82:IVar("m2"), 83:IAdd(InstructionI(59), I(InstructionI(60))), 84:IMul(InstructionI(82), I(InstructionI(83))), 85:IFuncSin(InstructionI(64)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IInv(InstructionI(81)), 88:IMul(InstructionI(86), I(InstructionI(87))), 89:IFuncCos(InstructionI(68)), 90:IMul(InstructionI(88), I(InstructionI(89))), 91:IVar("l2"), 92:IVar("omega2"), 93:IVar("theta2"), 94:IVar("theta1"), 95:INeg(InstructionI(93)), 96:IAdd(InstructionI(94), I(InstructionI(95))), 97:IFuncSin(InstructionI(96)), 98:INeg(InstructionI(91)), 99:IVar("m2"), 100:IMul(InstructionI(98), I(InstructionI(99))), 101:IExp { base: I(InstructionI(92)), power: C(2.0) }, 102:IMul(InstructionI(100), I(InstructionI(101))), 103:IExp { base: I(InstructionI(97)), power: C(2.0) }, 104:IVar("omega2"), 105:IVar("theta2"), 106:IVar("theta1"), 107:INeg(InstructionI(105)), 108:IAdd(InstructionI(106), I(InstructionI(107))), 109:IFuncCos(InstructionI(108)), 110:IVar("l2"), 111:IVar("m2"), 112:IMul(InstructionI(110), I(InstructionI(111))), 113:IExp { base: I(InstructionI(104)), power: C(2.0) }, 114:IMul(InstructionI(112), I(InstructionI(113))), 115:IExp { base: I(InstructionI(109)), power: C(2.0) }, 116:IVar("m1"), 117:IVar("m2"), 118:IVar("omega1"), 119:IVar("theta2"), 120:IVar("theta1"), 121:INeg(InstructionI(119)), 122:IAdd(InstructionI(120), I(InstructionI(121))), 123:IVar("l1"), 124:IExp { base: I(InstructionI(118)), power: C(2.0) }, 125:IMul(InstructionI(123), I(InstructionI(124))), 126:IFuncCos(InstructionI(122)), 127:IVar("theta1"), 128:IVar("theta2"), 129:IVar("theta1"), 130:INeg(InstructionI(128)), 131:IAdd(InstructionI(129), I(InstructionI(130))), 132:IFuncSin(InstructionI(127)), 133:IFuncSin(InstructionI(131)), 134:IMul(InstructionI(132), I(InstructionI(133))), 135:IMul(InstructionI(134), C(9.81)), 136:IVar("theta1"), 137:IVar("theta2"), 138:IVar("theta1"), 139:INeg(InstructionI(137)), 140:IAdd(InstructionI(138), I(InstructionI(139))), 141:IFuncCos(InstructionI(136)), 142:IFuncCos(InstructionI(140)), 143:IMul(InstructionI(141), I(InstructionI(142))), 144:INeg(InstructionI(135)), 145:IMul(InstructionI(125), I(InstructionI(126))), 146:IAdd(InstructionI(144), I(InstructionI(145))), 147:IMul(InstructionI(143), C(9.81)), 148:IAdd(InstructionI(116), I(InstructionI(117))), 149:IAdd(InstructionI(146), I(InstructionI(147))), 150:IMul(InstructionI(102), I(InstructionI(103))), 151:IMul(InstructionI(114), I(InstructionI(115))), 152:IAdd(InstructionI(150), I(InstructionI(151))), 153:IMul(InstructionI(148), I(InstructionI(149))), 154:IVar("theta2"), 155:IVar("theta1"), 156:INeg(InstructionI(154)), 157:IAdd(InstructionI(155), I(InstructionI(156))), 158:IFuncSin(InstructionI(157)), 159:IVar("m2"), 160:IExp { base: I(InstructionI(158)), power: C(2.0) }, 161:IVar("m1"), 162:IMul(InstructionI(159), I(InstructionI(160))), 163:IVar("l2"), 164:IAdd(InstructionI(161), I(InstructionI(162))), 165:IMul(InstructionI(163), I(InstructionI(164))), 166:IAdd(InstructionI(152), I(InstructionI(153))), 167:IInv(InstructionI(165)), 168:IMul(InstructionI(90), C(-2.0)), 169:IMul(InstructionI(166), I(InstructionI(167))) } }
CompileSlab{ instrs:{ 0:IVar("m1"), 1:IVar("m2"), 2:IVar("theta2"), 3:IVar("theta1"), 4:INeg(InstructionI(2)), 5:IAdd(InstructionI(3), I(InstructionI(4))), 6:IVar("theta2"), 7:IVar("theta1"), 8:INeg(InstructionI(6)), 9:IAdd(InstructionI(7), I(InstructionI(8))), 10:IFuncSin(InstructionI(9)), 11:IVar("m2"), 12:IExp { base: I(InstructionI(10)), power: C(2.0) }, 13:IVar("m1"), 14:IMul(InstructionI(11), I(InstructionI(12))), 15:IVar("l2"), 16:IAdd(InstructionI(13), I(InstructionI(14))), 17:IMul(InstructionI(15), I(InstructionI(16))), 18:IVar("l1"), 19:IVar("omega1"), 20:IMul(InstructionI(18), I(InstructionI(19))), 21:IAdd(InstructionI(0), I(InstructionI(1))), 22:IMul(InstructionI(20), I(InstructionI(21))), 23:IInv(InstructionI(17)), 24:IMul(InstructionI(22), I(InstructionI(23))), 25:IFuncSin(InstructionI(5)), 26:IMul(InstructionI(24), I(InstructionI(25))) } }
CompileSlab{ instrs:{ 0:IVar("air_resistance_coeff"), 1:IVar("omega2"), 2:IVar("omega2"), 3:IExp { base: I(InstructionI(2)), power: C(2.0) }, 4:IAdd(InstructionI(3), C(1e-10)), 5:IExp { base: I(InstructionI(4)), power: C(0.5) }, 6:INeg(InstructionI(0)), 7:IInv(InstructionI(5)), 8:IMul(InstructionI(6), I(InstructionI(7))), 9:IExp { base: I(InstructionI(1)), power: C(3.0) }, 10:IVar("omega2"), 11:IVar("theta2"), 12:IVar("theta1"), 13:INeg(InstructionI(11)), 14:IAdd(InstructionI(12), I(InstructionI(13))), 15:IVar("theta2"), 16:IVar("theta1"), 17:INeg(InstructionI(15)), 18:IAdd(InstructionI(16), I(InstructionI(17))), 19:IVar("l2"), 20:IVar("m2"), 21:IMul(InstructionI(19), I(InstructionI(20))), 22:IExp { base: I(InstructionI(10)), power: C(2.0) }, 23:IMul(InstructionI(21), I(InstructionI(22))), 24:IFuncSin(InstructionI(14)), 25:IMul(InstructionI(23), I(InstructionI(24))), 26:IFuncCos(InstructionI(18)), 27:IVar("m1"), 28:IVar("m2"), 29:IVar("omega1"), 30:IVar("theta2"), 31:IVar("theta1"), 32:INeg(InstructionI(30)), 33:IAdd(InstructionI(31), I(InstructionI(32))), 34:IVar("l1"), 35:IExp { base: I(InstructionI(29)), power: C(2.0) }, 36:IMul(InstructionI(34), I(InstructionI(35))), 37:IFuncSin(InstructionI(33)), 38:IVar("theta1"), 39:IVar("theta2"), 40:IVar("theta1"), 41:INeg(InstructionI(39)), 42:IAdd(InstructionI(40), I(InstructionI(41))), 43:IFuncSin(InstructionI(38)), 44:IFuncCos(InstructionI(42)), 45:IMul(InstructionI(43), I(InstructionI(44))), 46:IVar("theta2"), 47:IFuncSin(InstructionI(46)), 48:IMul(InstructionI(47), C(9.81)), 49:IMul(InstructionI(36), I(InstructionI(37))), 50:INeg(InstructionI(48)), 51:IAdd(InstructionI(49), I(InstructionI(50))), 52:IMul(InstructionI(45), C(9.81)), 53:IAdd(InstructionI(27), I(InstructionI(28))), 54:IAdd(InstructionI(51), I(InstructionI(52))), 55:IMul(InstructionI(8), I(InstructionI(9))), 56:IMul(InstructionI(25), I(InstructionI(26))), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IVar("u2"), 59:IAdd(InstructionI(57), I(InstructionI(58))), 60:IMul(InstructionI(53), I(InstructionI(54))), 61:IVar("theta2"), 62:IVar("theta1"), 63:INeg(InstructionI(61)), 64:IAdd(InstructionI(62), I(InstructionI(63))), 65:IVar("theta2"), 66:IVar("theta1"), 67:INeg(InstructionI(65)), 68:IAdd(InstructionI(66), I(InstructionI(67))), 69:IVar("theta2"), 70:IVar("theta1"), 71:INeg(InstructionI(69)), 72:IAdd(InstructionI(70), I(InstructionI(71))), 73:IFuncSin(InstructionI(72)), 74:IVar("m2"), 75:IExp { base: I(InstructionI(73)), power: C(2.0) }, 76:IVar("m1"), 77:IMul(InstructionI(74), I(InstructionI(75))), 78:IAdd(InstructionI(76), I(InstructionI(77))), 79:IVar("l2"), 80:IExp { base: I(InstructionI(78)), power: C(2.0) }, 81:IMul(InstructionI(79), I(InstructionI(80))), 82:IVar("m2"), 83:IAdd(InstructionI(59), I(InstructionI(60))), 84:IMul(InstructionI(82), I(InstructionI(83))), 85:IFuncSin(InstructionI(64)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IInv(InstructionI(81)), 88:IMul(InstructionI(86), I(InstructionI(87))), 89:IFuncCos(InstructionI(68)), 90:IMul(InstructionI(88), I(InstructionI(89))), 91:IVar("omega2"), 92:IVar("theta2"), 93:IVar("theta1"), 94:INeg(InstructionI(92)), 95:IAdd(InstructionI(93), I(InstructionI(94))), 96:IFuncSin(InstructionI(95)), 97:IVar("l2"), 98:IVar("m2"), 99:IMul(InstructionI(97), I(InstructionI(98))), 100:IExp { base: I(InstructionI(91)), power: C(2.0) }, 101:IMul(InstructionI(99), I(InstructionI(100))), 102:IExp { base: I(InstructionI(96)), power: C(2.0) }, 103:IVar("omega2"), 104:IVar("theta2"), 105:IVar("theta1"), 106:INeg(InstructionI(104)), 107:IAdd(InstructionI(105), I(InstructionI(106))), 108:IFuncCos(InstructionI(107)), 109:IVar("l2"), 110:IVar("m2"), 111:IMul(InstructionI(109), I(InstructionI(110))), 112:IExp { base: I(InstructionI(103)), power: C(2.0) }, 113:IMul(InstructionI(111), I(InstructionI(112))), 114:IExp { base: I(InstructionI(108)), power: C(2.0) }, 115:IMul(InstructionI(113), I(InstructionI(114))), 116:IVar("m1"), 117:IVar("m2"), 118:IVar("l1"), 119:IVar("omega1"), 120:IVar("theta2"), 121:IVar("theta1"), 122:INeg(InstructionI(120)), 123:IAdd(InstructionI(121), I(InstructionI(122))), 124:INeg(InstructionI(118)), 125:IExp { base: I(InstructionI(119)), power: C(2.0) }, 126:IMul(InstructionI(124), I(InstructionI(125))), 127:IFuncCos(InstructionI(123)), 128:IVar("theta1"), 129:IVar("theta2"), 130:IVar("theta1"), 131:INeg(InstructionI(129)), 132:IAdd(InstructionI(130), I(InstructionI(131))), 133:IFuncSin(InstructionI(128)), 134:IFuncSin(InstructionI(132)), 135:IMul(InstructionI(133), I(InstructionI(134))), 136:IVar("theta2"), 137:IFuncCos(InstructionI(136)), 138:IMul(InstructionI(137), C(9.81)), 139:IMul(InstructionI(126), I(InstructionI(127))), 140:INeg(InstructionI(138)), 141:IAdd(InstructionI(139), I(InstructionI(140))), 142:IMul(InstructionI(135), C(9.81)), 143:IAdd(InstructionI(116), I(InstructionI(117))), 144:IAdd(InstructionI(141), I(InstructionI(142))), 145:INeg(InstructionI(115)), 146:IMul(InstructionI(101), I(InstructionI(102))), 147:IAdd(InstructionI(145), I(InstructionI(146))), 148:IMul(InstructionI(143), I(InstructionI(144))), 149:IVar("theta2"), 150:IVar("theta1"), 151:INeg(InstructionI(149)), 152:IAdd(InstructionI(150), I(InstructionI(151))), 153:IFuncSin(InstructionI(152)), 154:IVar("m2"), 155:IExp { base: I(InstructionI(153)), power: C(2.0) }, 156:IVar("m1"), 157:IMul(InstructionI(154), I(InstructionI(155))), 158:IVar("l2"), 159:IAdd(InstructionI(156), I(InstructionI(157))), 160:IMul(InstructionI(158), I(InstructionI(159))), 161:IAdd(InstructionI(147), I(InstructionI(148))), 162:IInv(InstructionI(160)), 163:IMul(InstructionI(90), C(2.0)), 164:IMul(InstructionI(161), I(InstructionI(162))) } }
CompileSlab{ instrs:{ 0:IVar("omega2"), 1:IVar("omega2"), 2:IExp { base: I(InstructionI(1)), power: C(2.0) }, 3:IAdd(InstructionI(2), C(1e-10)), 4:IExp { base: I(InstructionI(3)), power: C(1.5) }, 5:IVar("air_resistance_coeff"), 6:IInv(InstructionI(4)), 7:IMul(InstructionI(5), I(InstructionI(6))), 8:IExp { base: I(InstructionI(0)), power: C(4.0) }, 9:IVar("omega2"), 10:IVar("omega2"), 11:IExp { base: I(InstructionI(10)), power: C(2.0) }, 12:IAdd(InstructionI(11), C(1e-10)), 13:IExp { base: I(InstructionI(12)), power: C(0.5) }, 14:IVar("air_resistance_coeff"), 15:IInv(InstructionI(13)), 16:IMul(InstructionI(14), I(InstructionI(15))), 17:IExp { base: I(InstructionI(9)), power: C(2.0) }, 18:IMul(InstructionI(16), I(InstructionI(17))), 19:IMul(InstructionI(18), C(3.0)), 20:IVar("theta2"), 21:IVar("theta1"), 22:INeg(InstructionI(20)), 23:IAdd(InstructionI(21), I(InstructionI(22))), 24:IVar("theta2"), 25:IVar("theta1"), 26:INeg(InstructionI(24)), 27:IAdd(InstructionI(25), I(InstructionI(26))), 28:IVar("l2"), 29:IVar("m2"), 30:IMul(InstructionI(28), I(InstructionI(29))), 31:IVar("omega2"), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IFuncSin(InstructionI(23)), 34:IMul(InstructionI(32), I(InstructionI(33))), 35:IFuncCos(InstructionI(27)), 36:IMul(InstructionI(34), I(InstructionI(35))), 37:INeg(InstructionI(19)), 38:IMul(InstructionI(7), I(InstructionI(8))), 39:IAdd(InstructionI(37), I(InstructionI(38))), 40:IMul(InstructionI(36), C(2.0)), 41:IVar("theta2"), 42:IVar("theta1"), 43:INeg(InstructionI(41)), 44:IAdd(InstructionI(42), I(InstructionI(43))), 45:IFuncSin(InstructionI(44)), 46:IVar("m2"), 47:IExp { base: I(InstructionI(45)), power: C(2.0) }, 48:IVar("m1"), 49:IMul(InstructionI(46), I(InstructionI(47))), 50:IVar("l2"), 51:IAdd(InstructionI(48), I(InstructionI(49))), 52:IMul(InstructionI(50), I(InstructionI(51))), 53:IAdd(InstructionI(39), I(InstructionI(40))), 54:IInv(InstructionI(52)) } }



  [[(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(1.0), Slab{ exprs:{ 0:Expression { first: EConstant(1.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} })], [(IAdd(InstructionI(163), I(InstructionI(164))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(1)))), ExprPair(EAdd, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 4:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 6:Expression { first: EConstant(9.81), pairs: [ExprPair(EMul, EStdFunc(EVar("m1"))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EVar("m2")))] }, 7:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 8:Expression { first: EUnaryOp(ENeg(ValueI(0))), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(3.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(0)))), ExprPair(EExp, EConstant(0.5)), ExprPair(ESub, EStdFunc(EVar("m2"))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(2)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(3)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(4)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(5)))), ExprPair(EAdd, EStdFunc(EVar("u1"))), ExprPair(ESub, EUnaryOp(EParentheses(ExpressionI(6)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(7))))] }, 9:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 10:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 11:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 12:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(11)))), ExprPair(EExp, EConstant(2.0))] }, 13:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(12)))), ExprPair(EExp, EConstant(2.0))] }, 14:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 15:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 16:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(15)))), ExprPair(EAdd, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0))] }, 17:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 18:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 19:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 20:Expression { first: EConstant(9.81), pairs: [ExprPair(EMul, EStdFunc(EVar("m1"))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EVar("m2")))] }, 21:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 22:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(14)))), ExprPair(EExp, EConstant(2.0)), ExprPair(ESub, EStdFunc(EVar("m2"))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(16)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(17)))), ExprPair(ESub, EConstant(9.81)), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(18)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(19)))), ExprPair(ESub, EUnaryOp(EParentheses(ExpressionI(20)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(21))))] }, 23:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 24:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(23)))), ExprPair(EExp, EConstant(2.0))] }, 25:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(24))))] }, 26:Expression { first: EConstant(-2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(8)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(9)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(10)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(13)))), ExprPair(EAdd, EUnaryOp(EParentheses(ExpressionI(22)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(25))))] } }, vals:{ 0:EStdFunc(EVar("air_resistance_coeff")) }, instrs:{ 0:IVar("air_resistance_coeff"), 1:IVar("omega1"), 2:IVar("omega1"), 3:IExp { base: I(InstructionI(2)), power: C(2.0) }, 4:IAdd(InstructionI(3), C(1e-10)), 5:IExp { base: I(InstructionI(4)), power: C(0.5) }, 6:INeg(InstructionI(0)), 7:IInv(InstructionI(5)), 8:IMul(InstructionI(6), I(InstructionI(7))), 9:IExp { base: I(InstructionI(1)), power: C(3.0) }, 10:IVar("omega1"), 11:IVar("theta2"), 12:IVar("theta1"), 13:INeg(InstructionI(11)), 14:IAdd(InstructionI(12), I(InstructionI(13))), 15:IVar("l1"), 16:IExp { base: I(InstructionI(10)), power: C(2.0) }, 17:IMul(InstructionI(15), I(InstructionI(16))), 18:IFuncCos(InstructionI(14)), 19:IVar("omega2"), 20:IVar("l2"), 21:IExp { base: I(InstructionI(19)), power: C(2.0) }, 22:IMul(InstructionI(17), I(InstructionI(18))), 23:IMul(InstructionI(20), I(InstructionI(21))), 24:IVar("theta2"), 25:IVar("theta1"), 26:INeg(InstructionI(24)), 27:IAdd(InstructionI(25), I(InstructionI(26))), 28:IVar("m2"), 29:IAdd(InstructionI(22), I(InstructionI(23))), 30:IMul(InstructionI(28), I(InstructionI(29))), 31:IFuncSin(InstructionI(27)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IVar("theta2"), 34:IVar("theta2"), 35:IVar("theta1"), 36:INeg(InstructionI(34)), 37:IAdd(InstructionI(35), I(InstructionI(36))), 38:IVar("m2"), 39:IFuncSin(InstructionI(33)), 40:IMul(InstructionI(38), I(InstructionI(39))), 41:IFuncCos(InstructionI(37)), 42:IMul(InstructionI(40), I(InstructionI(41))), 43:IVar("m1"), 44:IVar("m2"), 45:IMul(InstructionI(43), C(9.81)), 46:IMul(InstructionI(44), C(9.81)), 47:IVar("theta1"), 48:IAdd(InstructionI(45), I(InstructionI(46))), 49:IFuncSin(InstructionI(47)), 50:IMul(InstructionI(48), I(InstructionI(49))), 51:INeg(InstructionI(32)), 52:IMul(InstructionI(8), I(InstructionI(9))), 53:IAdd(InstructionI(51), I(InstructionI(52))), 54:IMul(InstructionI(42), C(9.81)), 55:IAdd(InstructionI(53), I(InstructionI(54))), 56:INeg(InstructionI(50)), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IVar("u1"), 59:IVar("theta2"), 60:IVar("theta1"), 61:INeg(InstructionI(59)), 62:IAdd(InstructionI(60), I(InstructionI(61))), 63:IVar("theta2"), 64:IVar("theta1"), 65:INeg(InstructionI(63)), 66:IAdd(InstructionI(64), I(InstructionI(65))), 67:IVar("theta2"), 68:IVar("theta1"), 69:INeg(InstructionI(67)), 70:IAdd(InstructionI(68), I(InstructionI(69))), 71:IFuncSin(InstructionI(70)), 72:IVar("m2"), 73:IExp { base: I(InstructionI(71)), power: C(2.0) }, 74:IVar("m1"), 75:IMul(InstructionI(72), I(InstructionI(73))), 76:IAdd(InstructionI(74), I(InstructionI(75))), 77:IVar("l1"), 78:IExp { base: I(InstructionI(76)), power: C(2.0) }, 79:IMul(InstructionI(77), I(InstructionI(78))), 80:IVar("m2"), 81:IAdd(InstructionI(57), I(InstructionI(58))), 82:IMul(InstructionI(80), I(InstructionI(81))), 83:IFuncSin(InstructionI(62)), 84:IMul(InstructionI(82), I(InstructionI(83))), 85:IInv(InstructionI(79)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IFuncCos(InstructionI(66)), 88:IMul(InstructionI(86), I(InstructionI(87))), 89:IVar("omega1"), 90:IVar("theta2"), 91:IVar("theta1"), 92:INeg(InstructionI(90)), 93:IAdd(InstructionI(91), I(InstructionI(92))), 94:IFuncSin(InstructionI(93)), 95:IVar("l1"), 96:IVar("m2"), 97:IMul(InstructionI(95), I(InstructionI(96))), 98:IExp { base: I(InstructionI(89)), power: C(2.0) }, 99:IMul(InstructionI(97), I(InstructionI(98))), 100:IExp { base: I(InstructionI(94)), power: C(2.0) }, 101:IVar("omega1"), 102:IVar("theta2"), 103:IVar("theta1"), 104:INeg(InstructionI(102)), 105:IAdd(InstructionI(103), I(InstructionI(104))), 106:IVar("l1"), 107:IExp { base: I(InstructionI(101)), power: C(2.0) }, 108:IMul(InstructionI(106), I(InstructionI(107))), 109:IFuncCos(InstructionI(105)), 110:IVar("omega2"), 111:IVar("l2"), 112:IExp { base: I(InstructionI(110)), power: C(2.0) }, 113:IMul(InstructionI(108), I(InstructionI(109))), 114:IMul(InstructionI(111), I(InstructionI(112))), 115:IVar("theta2"), 116:IVar("theta1"), 117:INeg(InstructionI(115)), 118:IAdd(InstructionI(116), I(InstructionI(117))), 119:IVar("m2"), 120:IAdd(InstructionI(113), I(InstructionI(114))), 121:IMul(InstructionI(119), I(InstructionI(120))), 122:IFuncCos(InstructionI(118)), 123:IMul(InstructionI(121), I(InstructionI(122))), 124:IVar("theta2"), 125:IVar("theta2"), 126:IVar("theta1"), 127:INeg(InstructionI(125)), 128:IAdd(InstructionI(126), I(InstructionI(127))), 129:IVar("m2"), 130:IFuncSin(InstructionI(124)), 131:IMul(InstructionI(129), I(InstructionI(130))), 132:IFuncSin(InstructionI(128)), 133:IMul(InstructionI(131), I(InstructionI(132))), 134:IMul(InstructionI(133), C(9.81)), 135:IVar("m1"), 136:IVar("m2"), 137:IMul(InstructionI(135), C(9.81)), 138:IMul(InstructionI(136), C(9.81)), 139:IVar("theta1"), 140:IAdd(InstructionI(137), I(InstructionI(138))), 141:IFuncCos(InstructionI(139)), 142:IMul(InstructionI(140), I(InstructionI(141))), 143:IMul(InstructionI(99), I(InstructionI(100))), 144:INeg(InstructionI(123)), 145:IAdd(InstructionI(143), I(InstructionI(144))), 146:INeg(InstructionI(134)), 147:IAdd(InstructionI(145), I(InstructionI(146))), 148:INeg(InstructionI(142)), 149:IVar("theta2"), 150:IVar("theta1"), 151:INeg(InstructionI(149)), 152:IAdd(InstructionI(150), I(InstructionI(151))), 153:IFuncSin(InstructionI(152)), 154:IVar("m2"), 155:IExp { base: I(InstructionI(153)), power: C(2.0) }, 156:IVar("m1"), 157:IMul(InstructionI(154), I(InstructionI(155))), 158:IVar("l1"), 159:IAdd(InstructionI(156), I(InstructionI(157))), 160:IMul(InstructionI(158), I(InstructionI(159))), 161:IAdd(InstructionI(147), I(InstructionI(148))), 162:IInv(InstructionI(160)), 163:IMul(InstructionI(88), C(-2.0)), 164:IMul(InstructionI(161), I(InstructionI(162))) } }), (IMul(InstructionI(54), I(InstructionI(55))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 1:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 2:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 4:Expression { first: EConstant(1.0), pairs: [ExprPair(EMul, EStdFunc(EVar("air_resistance_coeff"))), ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(4.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(0)))), ExprPair(EExp, EConstant(1.5)), ExprPair(ESub, EConstant(3.0)), ExprPair(EMul, EStdFunc(EVar("air_resistance_coeff"))), ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(1)))), ExprPair(EExp, EConstant(0.5)), ExprPair(ESub, EConstant(2.0)), ExprPair(EMul, EStdFunc(EVar("l1"))), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(2)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(3))))] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 6:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(5)))), ExprPair(EExp, EConstant(2.0))] }, 7:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(6))))] }, 8:Expression { first: EUnaryOp(EParentheses(ExpressionI(4))), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(7))))] } }, vals:{}, instrs:{ 0:IVar("omega1"), 1:IVar("omega1"), 2:IExp { base: I(InstructionI(1)), power: C(2.0) }, 3:IAdd(InstructionI(2), C(1e-10)), 4:IExp { base: I(InstructionI(3)), power: C(1.5) }, 5:IVar("air_resistance_coeff"), 6:IInv(InstructionI(4)), 7:IMul(InstructionI(5), I(InstructionI(6))), 8:IExp { base: I(InstructionI(0)), power: C(4.0) }, 9:IVar("omega1"), 10:IVar("omega1"), 11:IExp { base: I(InstructionI(10)), power: C(2.0) }, 12:IAdd(InstructionI(11), C(1e-10)), 13:IExp { base: I(InstructionI(12)), power: C(0.5) }, 14:IVar("air_resistance_coeff"), 15:IInv(InstructionI(13)), 16:IMul(InstructionI(14), I(InstructionI(15))), 17:IExp { base: I(InstructionI(9)), power: C(2.0) }, 18:IMul(InstructionI(16), I(InstructionI(17))), 19:IMul(InstructionI(18), C(3.0)), 20:IVar("theta2"), 21:IVar("theta1"), 22:INeg(InstructionI(20)), 23:IAdd(InstructionI(21), I(InstructionI(22))), 24:IVar("theta2"), 25:IVar("theta1"), 26:INeg(InstructionI(24)), 27:IAdd(InstructionI(25), I(InstructionI(26))), 28:IVar("l1"), 29:IVar("m2"), 30:IMul(InstructionI(28), I(InstructionI(29))), 31:IVar("omega1"), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IFuncSin(InstructionI(23)), 34:IMul(InstructionI(32), I(InstructionI(33))), 35:IFuncCos(InstructionI(27)), 36:IMul(InstructionI(34), I(InstructionI(35))), 37:IMul(InstructionI(36), C(2.0)), 38:IMul(InstructionI(7), I(InstructionI(8))), 39:INeg(InstructionI(19)), 40:IAdd(InstructionI(38), I(InstructionI(39))), 41:INeg(InstructionI(37)), 42:IVar("theta2"), 43:IVar("theta1"), 44:INeg(InstructionI(42)), 45:IAdd(InstructionI(43), I(InstructionI(44))), 46:IFuncSin(InstructionI(45)), 47:IVar("m2"), 48:IExp { base: I(InstructionI(46)), power: C(2.0) }, 49:IVar("m1"), 50:IMul(InstructionI(47), I(InstructionI(48))), 51:IVar("l1"), 52:IAdd(InstructionI(49), I(InstructionI(50))), 53:IMul(InstructionI(51), I(InstructionI(52))), 54:IAdd(InstructionI(40), I(InstructionI(41))), 55:IInv(InstructionI(53)) } }), (IAdd(InstructionI(164), I(InstructionI(165))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("omega1")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(1)))), ExprPair(EAdd, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 4:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 6:Expression { first: EConstant(9.81), pairs: [ExprPair(EMul, EStdFunc(EVar("m1"))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EVar("m2")))] }, 7:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 8:Expression { first: EUnaryOp(ENeg(ValueI(0))), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(3.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(0)))), ExprPair(EExp, EConstant(0.5)), ExprPair(ESub, EStdFunc(EVar("m2"))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(2)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(3)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(4)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(5)))), ExprPair(EAdd, EStdFunc(EVar("u1"))), ExprPair(ESub, EUnaryOp(EParentheses(ExpressionI(6)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(7))))] }, 9:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 10:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 11:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 12:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(11)))), ExprPair(EExp, EConstant(2.0))] }, 13:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(12)))), ExprPair(EExp, EConstant(2.0))] }, 14:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 15:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 16:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(15)))), ExprPair(EAdd, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0))] }, 17:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 18:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 19:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 20:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 21:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 22:Expression { first: EUnaryOp(ENeg(ValueI(1))), pairs: [ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(14)))), ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(16)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(17)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(18)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(19)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(20)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(21))))] }, 23:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 24:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(23)))), ExprPair(EExp, EConstant(2.0))] }, 25:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(24))))] }, 26:Expression { first: EConstant(2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(8)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(9)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(10)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(13)))), ExprPair(EAdd, EUnaryOp(EParentheses(ExpressionI(22)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(25))))] } }, vals:{ 0:EStdFunc(EVar("air_resistance_coeff")), 1:EStdFunc(EVar("l1")) }, instrs:{ 0:IVar("air_resistance_coeff"), 1:IVar("omega1"), 2:IVar("omega1"), 3:IExp { base: I(InstructionI(2)), power: C(2.0) }, 4:IAdd(InstructionI(3), C(1e-10)), 5:IExp { base: I(InstructionI(4)), power: C(0.5) }, 6:INeg(InstructionI(0)), 7:IInv(InstructionI(5)), 8:IMul(InstructionI(6), I(InstructionI(7))), 9:IExp { base: I(InstructionI(1)), power: C(3.0) }, 10:IVar("omega1"), 11:IVar("theta2"), 12:IVar("theta1"), 13:INeg(InstructionI(11)), 14:IAdd(InstructionI(12), I(InstructionI(13))), 15:IVar("l1"), 16:IExp { base: I(InstructionI(10)), power: C(2.0) }, 17:IMul(InstructionI(15), I(InstructionI(16))), 18:IFuncCos(InstructionI(14)), 19:IVar("omega2"), 20:IVar("l2"), 21:IExp { base: I(InstructionI(19)), power: C(2.0) }, 22:IMul(InstructionI(17), I(InstructionI(18))), 23:IMul(InstructionI(20), I(InstructionI(21))), 24:IVar("theta2"), 25:IVar("theta1"), 26:INeg(InstructionI(24)), 27:IAdd(InstructionI(25), I(InstructionI(26))), 28:IVar("m2"), 29:IAdd(InstructionI(22), I(InstructionI(23))), 30:IMul(InstructionI(28), I(InstructionI(29))), 31:IFuncSin(InstructionI(27)), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IVar("theta2"), 34:IVar("theta2"), 35:IVar("theta1"), 36:INeg(InstructionI(34)), 37:IAdd(InstructionI(35), I(InstructionI(36))), 38:IVar("m2"), 39:IFuncSin(InstructionI(33)), 40:IMul(InstructionI(38), I(InstructionI(39))), 41:IFuncCos(InstructionI(37)), 42:IMul(InstructionI(40), I(InstructionI(41))), 43:IVar("m1"), 44:IVar("m2"), 45:IMul(InstructionI(43), C(9.81)), 46:IMul(InstructionI(44), C(9.81)), 47:IVar("theta1"), 48:IAdd(InstructionI(45), I(InstructionI(46))), 49:IFuncSin(InstructionI(47)), 50:IMul(InstructionI(48), I(InstructionI(49))), 51:INeg(InstructionI(32)), 52:IMul(InstructionI(8), I(InstructionI(9))), 53:IAdd(InstructionI(51), I(InstructionI(52))), 54:IMul(InstructionI(42), C(9.81)), 55:IAdd(InstructionI(53), I(InstructionI(54))), 56:INeg(InstructionI(50)), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IVar("u1"), 59:IVar("theta2"), 60:IVar("theta1"), 61:INeg(InstructionI(59)), 62:IAdd(InstructionI(60), I(InstructionI(61))), 63:IVar("theta2"), 64:IVar("theta1"), 65:INeg(InstructionI(63)), 66:IAdd(InstructionI(64), I(InstructionI(65))), 67:IVar("theta2"), 68:IVar("theta1"), 69:INeg(InstructionI(67)), 70:IAdd(InstructionI(68), I(InstructionI(69))), 71:IFuncSin(InstructionI(70)), 72:IVar("m2"), 73:IExp { base: I(InstructionI(71)), power: C(2.0) }, 74:IVar("m1"), 75:IMul(InstructionI(72), I(InstructionI(73))), 76:IAdd(InstructionI(74), I(InstructionI(75))), 77:IVar("l1"), 78:IExp { base: I(InstructionI(76)), power: C(2.0) }, 79:IMul(InstructionI(77), I(InstructionI(78))), 80:IVar("m2"), 81:IAdd(InstructionI(57), I(InstructionI(58))), 82:IMul(InstructionI(80), I(InstructionI(81))), 83:IFuncSin(InstructionI(62)), 84:IMul(InstructionI(82), I(InstructionI(83))), 85:IInv(InstructionI(79)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IFuncCos(InstructionI(66)), 88:IMul(InstructionI(86), I(InstructionI(87))), 89:IVar("l1"), 90:IVar("omega1"), 91:IVar("theta2"), 92:IVar("theta1"), 93:INeg(InstructionI(91)), 94:IAdd(InstructionI(92), I(InstructionI(93))), 95:IFuncSin(InstructionI(94)), 96:INeg(InstructionI(89)), 97:IVar("m2"), 98:IMul(InstructionI(96), I(InstructionI(97))), 99:IExp { base: I(InstructionI(90)), power: C(2.0) }, 100:IMul(InstructionI(98), I(InstructionI(99))), 101:IExp { base: I(InstructionI(95)), power: C(2.0) }, 102:IVar("omega1"), 103:IVar("theta2"), 104:IVar("theta1"), 105:INeg(InstructionI(103)), 106:IAdd(InstructionI(104), I(InstructionI(105))), 107:IVar("l1"), 108:IExp { base: I(InstructionI(102)), power: C(2.0) }, 109:IMul(InstructionI(107), I(InstructionI(108))), 110:IFuncCos(InstructionI(106)), 111:IVar("omega2"), 112:IVar("l2"), 113:IExp { base: I(InstructionI(111)), power: C(2.0) }, 114:IMul(InstructionI(109), I(InstructionI(110))), 115:IMul(InstructionI(112), I(InstructionI(113))), 116:IVar("theta2"), 117:IVar("theta1"), 118:INeg(InstructionI(116)), 119:IAdd(InstructionI(117), I(InstructionI(118))), 120:IVar("m2"), 121:IAdd(InstructionI(114), I(InstructionI(115))), 122:IMul(InstructionI(120), I(InstructionI(121))), 123:IFuncCos(InstructionI(119)), 124:IVar("theta2"), 125:IVar("theta2"), 126:IVar("theta1"), 127:INeg(InstructionI(125)), 128:IAdd(InstructionI(126), I(InstructionI(127))), 129:IVar("m2"), 130:IFuncSin(InstructionI(124)), 131:IMul(InstructionI(129), I(InstructionI(130))), 132:IFuncSin(InstructionI(128)), 133:IMul(InstructionI(131), I(InstructionI(132))), 134:IVar("theta2"), 135:IVar("theta2"), 136:IVar("theta1"), 137:INeg(InstructionI(135)), 138:IAdd(InstructionI(136), I(InstructionI(137))), 139:IVar("m2"), 140:IFuncCos(InstructionI(134)), 141:IMul(InstructionI(139), I(InstructionI(140))), 142:IFuncCos(InstructionI(138)), 143:IMul(InstructionI(141), I(InstructionI(142))), 144:IMul(InstructionI(100), I(InstructionI(101))), 145:IMul(InstructionI(122), I(InstructionI(123))), 146:IAdd(InstructionI(144), I(InstructionI(145))), 147:IMul(InstructionI(133), C(9.81)), 148:IAdd(InstructionI(146), I(InstructionI(147))), 149:IMul(InstructionI(143), C(9.81)), 150:IVar("theta2"), 151:IVar("theta1"), 152:INeg(InstructionI(150)), 153:IAdd(InstructionI(151), I(InstructionI(152))), 154:IFuncSin(InstructionI(153)), 155:IVar("m2"), 156:IExp { base: I(InstructionI(154)), power: C(2.0) }, 157:IVar("m1"), 158:IMul(InstructionI(155), I(InstructionI(156))), 159:IVar("l1"), 160:IAdd(InstructionI(157), I(InstructionI(158))), 161:IMul(InstructionI(159), I(InstructionI(160))), 162:IAdd(InstructionI(148), I(InstructionI(149))), 163:IInv(InstructionI(161)), 164:IMul(InstructionI(88), C(2.0)), 165:IMul(InstructionI(162), I(InstructionI(163))) } }), (IMul(InstructionI(24), C(-2.0)), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(1)))), ExprPair(EExp, EConstant(2.0))] }, 3:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(2))))] }, 4:Expression { first: EConstant(-2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(0)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(3))))] } }, vals:{}, instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IVar("theta2"), 5:IVar("theta1"), 6:INeg(InstructionI(4)), 7:IAdd(InstructionI(5), I(InstructionI(6))), 8:IFuncSin(InstructionI(7)), 9:IVar("m2"), 10:IExp { base: I(InstructionI(8)), power: C(2.0) }, 11:IVar("m1"), 12:IMul(InstructionI(9), I(InstructionI(10))), 13:IVar("l1"), 14:IAdd(InstructionI(11), I(InstructionI(12))), 15:IMul(InstructionI(13), I(InstructionI(14))), 16:IVar("l2"), 17:IVar("m2"), 18:IMul(InstructionI(16), I(InstructionI(17))), 19:IVar("omega2"), 20:IMul(InstructionI(18), I(InstructionI(19))), 21:IInv(InstructionI(15)), 22:IMul(InstructionI(20), I(InstructionI(21))), 23:IFuncSin(InstructionI(3)), 24:IMul(InstructionI(22), I(InstructionI(23))) } })], [(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(1.0), Slab{ exprs:{ 0:Expression { first: EConstant(1.0), pairs: [] } }, vals:{}, instrs:{} })], [(IAdd(InstructionI(168), I(InstructionI(169))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("omega2")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 3:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2")))] }, 4:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 6:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 7:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 8:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(4)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(5)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(6)))), ExprPair(ESub, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(7))))] }, 9:Expression { first: EUnaryOp(ENeg(ValueI(0))), pairs: [ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(3.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(0)))), ExprPair(EExp, EConstant(0.5)), ExprPair(EAdd, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(1)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(2)))), ExprPair(EAdd, EStdFunc(EVar("u2"))), ExprPair(EAdd, EUnaryOp(EParentheses(ExpressionI(3)))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(8))))] }, 10:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 11:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 12:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 13:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(12)))), ExprPair(EExp, EConstant(2.0))] }, 14:Expression { first: EStdFunc(EVar("l2")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(13)))), ExprPair(EExp, EConstant(2.0))] }, 15:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 16:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 17:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2")))] }, 18:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 19:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 20:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 21:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 22:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 23:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(18)))), ExprPair(ESub, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(19)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(20)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(21)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(22))))] }, 24:Expression { first: EUnaryOp(ENeg(ValueI(1))), pairs: [ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(15)))), ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(16)))), ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EUnaryOp(EParentheses(ExpressionI(17)))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(23))))] }, 25:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 26:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(25)))), ExprPair(EExp, EConstant(2.0))] }, 27:Expression { first: EStdFunc(EVar("l2")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(26))))] }, 28:Expression { first: EConstant(-2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(9)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(10)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(11)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(14)))), ExprPair(EAdd, EUnaryOp(EParentheses(ExpressionI(24)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(27))))] } }, vals:{ 0:EStdFunc(EVar("air_resistance_coeff")), 1:EStdFunc(EVar("l2")) }, instrs:{ 0:IVar("air_resistance_coeff"), 1:IVar("omega2"), 2:IVar("omega2"), 3:IExp { base: I(InstructionI(2)), power: C(2.0) }, 4:IAdd(InstructionI(3), C(1e-10)), 5:IExp { base: I(InstructionI(4)), power: C(0.5) }, 6:INeg(InstructionI(0)), 7:IInv(InstructionI(5)), 8:IMul(InstructionI(6), I(InstructionI(7))), 9:IExp { base: I(InstructionI(1)), power: C(3.0) }, 10:IVar("omega2"), 11:IVar("theta2"), 12:IVar("theta1"), 13:INeg(InstructionI(11)), 14:IAdd(InstructionI(12), I(InstructionI(13))), 15:IVar("theta2"), 16:IVar("theta1"), 17:INeg(InstructionI(15)), 18:IAdd(InstructionI(16), I(InstructionI(17))), 19:IVar("l2"), 20:IVar("m2"), 21:IMul(InstructionI(19), I(InstructionI(20))), 22:IExp { base: I(InstructionI(10)), power: C(2.0) }, 23:IMul(InstructionI(21), I(InstructionI(22))), 24:IFuncSin(InstructionI(14)), 25:IMul(InstructionI(23), I(InstructionI(24))), 26:IFuncCos(InstructionI(18)), 27:IVar("m1"), 28:IVar("m2"), 29:IVar("omega1"), 30:IVar("theta2"), 31:IVar("theta1"), 32:INeg(InstructionI(30)), 33:IAdd(InstructionI(31), I(InstructionI(32))), 34:IVar("l1"), 35:IExp { base: I(InstructionI(29)), power: C(2.0) }, 36:IMul(InstructionI(34), I(InstructionI(35))), 37:IFuncSin(InstructionI(33)), 38:IVar("theta1"), 39:IVar("theta2"), 40:IVar("theta1"), 41:INeg(InstructionI(39)), 42:IAdd(InstructionI(40), I(InstructionI(41))), 43:IFuncSin(InstructionI(38)), 44:IFuncCos(InstructionI(42)), 45:IMul(InstructionI(43), I(InstructionI(44))), 46:IVar("theta2"), 47:IFuncSin(InstructionI(46)), 48:IMul(InstructionI(47), C(9.81)), 49:IMul(InstructionI(36), I(InstructionI(37))), 50:INeg(InstructionI(48)), 51:IAdd(InstructionI(49), I(InstructionI(50))), 52:IMul(InstructionI(45), C(9.81)), 53:IAdd(InstructionI(27), I(InstructionI(28))), 54:IAdd(InstructionI(51), I(InstructionI(52))), 55:IMul(InstructionI(8), I(InstructionI(9))), 56:IMul(InstructionI(25), I(InstructionI(26))), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IVar("u2"), 59:IAdd(InstructionI(57), I(InstructionI(58))), 60:IMul(InstructionI(53), I(InstructionI(54))), 61:IVar("theta2"), 62:IVar("theta1"), 63:INeg(InstructionI(61)), 64:IAdd(InstructionI(62), I(InstructionI(63))), 65:IVar("theta2"), 66:IVar("theta1"), 67:INeg(InstructionI(65)), 68:IAdd(InstructionI(66), I(InstructionI(67))), 69:IVar("theta2"), 70:IVar("theta1"), 71:INeg(InstructionI(69)), 72:IAdd(InstructionI(70), I(InstructionI(71))), 73:IFuncSin(InstructionI(72)), 74:IVar("m2"), 75:IExp { base: I(InstructionI(73)), power: C(2.0) }, 76:IVar("m1"), 77:IMul(InstructionI(74), I(InstructionI(75))), 78:IAdd(InstructionI(76), I(InstructionI(77))), 79:IVar("l2"), 80:IExp { base: I(InstructionI(78)), power: C(2.0) }, 81:IMul(InstructionI(79), I(InstructionI(80))), 82:IVar("m2"), 83:IAdd(InstructionI(59), I(InstructionI(60))), 84:IMul(InstructionI(82), I(InstructionI(83))), 85:IFuncSin(InstructionI(64)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IInv(InstructionI(81)), 88:IMul(InstructionI(86), I(InstructionI(87))), 89:IFuncCos(InstructionI(68)), 90:IMul(InstructionI(88), I(InstructionI(89))), 91:IVar("l2"), 92:IVar("omega2"), 93:IVar("theta2"), 94:IVar("theta1"), 95:INeg(InstructionI(93)), 96:IAdd(InstructionI(94), I(InstructionI(95))), 97:IFuncSin(InstructionI(96)), 98:INeg(InstructionI(91)), 99:IVar("m2"), 100:IMul(InstructionI(98), I(InstructionI(99))), 101:IExp { base: I(InstructionI(92)), power: C(2.0) }, 102:IMul(InstructionI(100), I(InstructionI(101))), 103:IExp { base: I(InstructionI(97)), power: C(2.0) }, 104:IVar("omega2"), 105:IVar("theta2"), 106:IVar("theta1"), 107:INeg(InstructionI(105)), 108:IAdd(InstructionI(106), I(InstructionI(107))), 109:IFuncCos(InstructionI(108)), 110:IVar("l2"), 111:IVar("m2"), 112:IMul(InstructionI(110), I(InstructionI(111))), 113:IExp { base: I(InstructionI(104)), power: C(2.0) }, 114:IMul(InstructionI(112), I(InstructionI(113))), 115:IExp { base: I(InstructionI(109)), power: C(2.0) }, 116:IVar("m1"), 117:IVar("m2"), 118:IVar("omega1"), 119:IVar("theta2"), 120:IVar("theta1"), 121:INeg(InstructionI(119)), 122:IAdd(InstructionI(120), I(InstructionI(121))), 123:IVar("l1"), 124:IExp { base: I(InstructionI(118)), power: C(2.0) }, 125:IMul(InstructionI(123), I(InstructionI(124))), 126:IFuncCos(InstructionI(122)), 127:IVar("theta1"), 128:IVar("theta2"), 129:IVar("theta1"), 130:INeg(InstructionI(128)), 131:IAdd(InstructionI(129), I(InstructionI(130))), 132:IFuncSin(InstructionI(127)), 133:IFuncSin(InstructionI(131)), 134:IMul(InstructionI(132), I(InstructionI(133))), 135:IMul(InstructionI(134), C(9.81)), 136:IVar("theta1"), 137:IVar("theta2"), 138:IVar("theta1"), 139:INeg(InstructionI(137)), 140:IAdd(InstructionI(138), I(InstructionI(139))), 141:IFuncCos(InstructionI(136)), 142:IFuncCos(InstructionI(140)), 143:IMul(InstructionI(141), I(InstructionI(142))), 144:INeg(InstructionI(135)), 145:IMul(InstructionI(125), I(InstructionI(126))), 146:IAdd(InstructionI(144), I(InstructionI(145))), 147:IMul(InstructionI(143), C(9.81)), 148:IAdd(InstructionI(116), I(InstructionI(117))), 149:IAdd(InstructionI(146), I(InstructionI(147))), 150:IMul(InstructionI(102), I(InstructionI(103))), 151:IMul(InstructionI(114), I(InstructionI(115))), 152:IAdd(InstructionI(150), I(InstructionI(151))), 153:IMul(InstructionI(148), I(InstructionI(149))), 154:IVar("theta2"), 155:IVar("theta1"), 156:INeg(InstructionI(154)), 157:IAdd(InstructionI(155), I(InstructionI(156))), 158:IFuncSin(InstructionI(157)), 159:IVar("m2"), 160:IExp { base: I(InstructionI(158)), power: C(2.0) }, 161:IVar("m1"), 162:IMul(InstructionI(159), I(InstructionI(160))), 163:IVar("l2"), 164:IAdd(InstructionI(161), I(InstructionI(162))), 165:IMul(InstructionI(163), I(InstructionI(164))), 166:IAdd(InstructionI(152), I(InstructionI(153))), 167:IInv(InstructionI(165)), 168:IMul(InstructionI(90), C(-2.0)), 169:IMul(InstructionI(166), I(InstructionI(167))) } }), (IMul(InstructionI(26), C(2.0)), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2")))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 3:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(2)))), ExprPair(EExp, EConstant(2.0))] }, 4:Expression { first: EStdFunc(EVar("l2")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(3))))] }, 5:Expression { first: EConstant(2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("l1"))), ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(0)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(1)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(4))))] } }, vals:{}, instrs:{ 0:IVar("m1"), 1:IVar("m2"), 2:IVar("theta2"), 3:IVar("theta1"), 4:INeg(InstructionI(2)), 5:IAdd(InstructionI(3), I(InstructionI(4))), 6:IVar("theta2"), 7:IVar("theta1"), 8:INeg(InstructionI(6)), 9:IAdd(InstructionI(7), I(InstructionI(8))), 10:IFuncSin(InstructionI(9)), 11:IVar("m2"), 12:IExp { base: I(InstructionI(10)), power: C(2.0) }, 13:IVar("m1"), 14:IMul(InstructionI(11), I(InstructionI(12))), 15:IVar("l2"), 16:IAdd(InstructionI(13), I(InstructionI(14))), 17:IMul(InstructionI(15), I(InstructionI(16))), 18:IVar("l1"), 19:IVar("omega1"), 20:IMul(InstructionI(18), I(InstructionI(19))), 21:IAdd(InstructionI(0), I(InstructionI(1))), 22:IMul(InstructionI(20), I(InstructionI(21))), 23:IInv(InstructionI(17)), 24:IMul(InstructionI(22), I(InstructionI(23))), 25:IFuncSin(InstructionI(5)), 26:IMul(InstructionI(24), I(InstructionI(25))) } }), (IAdd(InstructionI(163), I(InstructionI(164))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("omega2")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 1:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 2:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 3:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2")))] }, 4:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 6:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 7:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 8:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(4)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(5)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(6)))), ExprPair(ESub, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(7))))] }, 9:Expression { first: EUnaryOp(ENeg(ValueI(0))), pairs: [ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(3.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(0)))), ExprPair(EExp, EConstant(0.5)), ExprPair(EAdd, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(1)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(2)))), ExprPair(EAdd, EStdFunc(EVar("u2"))), ExprPair(EAdd, EUnaryOp(EParentheses(ExpressionI(3)))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(8))))] }, 10:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 11:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 12:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 13:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(12)))), ExprPair(EExp, EConstant(2.0))] }, 14:Expression { first: EStdFunc(EVar("l2")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(13)))), ExprPair(EExp, EConstant(2.0))] }, 15:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 16:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 17:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2")))] }, 18:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 19:Expression { first: EStdFunc(EVar("theta1")), pairs: [] }, 20:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 21:Expression { first: EStdFunc(EVar("theta2")), pairs: [] }, 22:Expression { first: EUnaryOp(ENeg(ValueI(1))), pairs: [ExprPair(EMul, EStdFunc(EVar("omega1"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(18)))), ExprPair(EAdd, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(19)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(20)))), ExprPair(ESub, EConstant(9.81)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(21))))] }, 23:Expression { first: EStdFunc(EVar("l2")), pairs: [ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(15)))), ExprPair(EExp, EConstant(2.0)), ExprPair(ESub, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(16)))), ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EUnaryOp(EParentheses(ExpressionI(17)))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(22))))] }, 24:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 25:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(24)))), ExprPair(EExp, EConstant(2.0))] }, 26:Expression { first: EStdFunc(EVar("l2")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(25))))] }, 27:Expression { first: EConstant(2.0), pairs: [ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(9)))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(10)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(11)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(14)))), ExprPair(EAdd, EUnaryOp(EParentheses(ExpressionI(23)))), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(26))))] } }, vals:{ 0:EStdFunc(EVar("air_resistance_coeff")), 1:EStdFunc(EVar("l1")) }, instrs:{ 0:IVar("air_resistance_coeff"), 1:IVar("omega2"), 2:IVar("omega2"), 3:IExp { base: I(InstructionI(2)), power: C(2.0) }, 4:IAdd(InstructionI(3), C(1e-10)), 5:IExp { base: I(InstructionI(4)), power: C(0.5) }, 6:INeg(InstructionI(0)), 7:IInv(InstructionI(5)), 8:IMul(InstructionI(6), I(InstructionI(7))), 9:IExp { base: I(InstructionI(1)), power: C(3.0) }, 10:IVar("omega2"), 11:IVar("theta2"), 12:IVar("theta1"), 13:INeg(InstructionI(11)), 14:IAdd(InstructionI(12), I(InstructionI(13))), 15:IVar("theta2"), 16:IVar("theta1"), 17:INeg(InstructionI(15)), 18:IAdd(InstructionI(16), I(InstructionI(17))), 19:IVar("l2"), 20:IVar("m2"), 21:IMul(InstructionI(19), I(InstructionI(20))), 22:IExp { base: I(InstructionI(10)), power: C(2.0) }, 23:IMul(InstructionI(21), I(InstructionI(22))), 24:IFuncSin(InstructionI(14)), 25:IMul(InstructionI(23), I(InstructionI(24))), 26:IFuncCos(InstructionI(18)), 27:IVar("m1"), 28:IVar("m2"), 29:IVar("omega1"), 30:IVar("theta2"), 31:IVar("theta1"), 32:INeg(InstructionI(30)), 33:IAdd(InstructionI(31), I(InstructionI(32))), 34:IVar("l1"), 35:IExp { base: I(InstructionI(29)), power: C(2.0) }, 36:IMul(InstructionI(34), I(InstructionI(35))), 37:IFuncSin(InstructionI(33)), 38:IVar("theta1"), 39:IVar("theta2"), 40:IVar("theta1"), 41:INeg(InstructionI(39)), 42:IAdd(InstructionI(40), I(InstructionI(41))), 43:IFuncSin(InstructionI(38)), 44:IFuncCos(InstructionI(42)), 45:IMul(InstructionI(43), I(InstructionI(44))), 46:IVar("theta2"), 47:IFuncSin(InstructionI(46)), 48:IMul(InstructionI(47), C(9.81)), 49:IMul(InstructionI(36), I(InstructionI(37))), 50:INeg(InstructionI(48)), 51:IAdd(InstructionI(49), I(InstructionI(50))), 52:IMul(InstructionI(45), C(9.81)), 53:IAdd(InstructionI(27), I(InstructionI(28))), 54:IAdd(InstructionI(51), I(InstructionI(52))), 55:IMul(InstructionI(8), I(InstructionI(9))), 56:IMul(InstructionI(25), I(InstructionI(26))), 57:IAdd(InstructionI(55), I(InstructionI(56))), 58:IVar("u2"), 59:IAdd(InstructionI(57), I(InstructionI(58))), 60:IMul(InstructionI(53), I(InstructionI(54))), 61:IVar("theta2"), 62:IVar("theta1"), 63:INeg(InstructionI(61)), 64:IAdd(InstructionI(62), I(InstructionI(63))), 65:IVar("theta2"), 66:IVar("theta1"), 67:INeg(InstructionI(65)), 68:IAdd(InstructionI(66), I(InstructionI(67))), 69:IVar("theta2"), 70:IVar("theta1"), 71:INeg(InstructionI(69)), 72:IAdd(InstructionI(70), I(InstructionI(71))), 73:IFuncSin(InstructionI(72)), 74:IVar("m2"), 75:IExp { base: I(InstructionI(73)), power: C(2.0) }, 76:IVar("m1"), 77:IMul(InstructionI(74), I(InstructionI(75))), 78:IAdd(InstructionI(76), I(InstructionI(77))), 79:IVar("l2"), 80:IExp { base: I(InstructionI(78)), power: C(2.0) }, 81:IMul(InstructionI(79), I(InstructionI(80))), 82:IVar("m2"), 83:IAdd(InstructionI(59), I(InstructionI(60))), 84:IMul(InstructionI(82), I(InstructionI(83))), 85:IFuncSin(InstructionI(64)), 86:IMul(InstructionI(84), I(InstructionI(85))), 87:IInv(InstructionI(81)), 88:IMul(InstructionI(86), I(InstructionI(87))), 89:IFuncCos(InstructionI(68)), 90:IMul(InstructionI(88), I(InstructionI(89))), 91:IVar("omega2"), 92:IVar("theta2"), 93:IVar("theta1"), 94:INeg(InstructionI(92)), 95:IAdd(InstructionI(93), I(InstructionI(94))), 96:IFuncSin(InstructionI(95)), 97:IVar("l2"), 98:IVar("m2"), 99:IMul(InstructionI(97), I(InstructionI(98))), 100:IExp { base: I(InstructionI(91)), power: C(2.0) }, 101:IMul(InstructionI(99), I(InstructionI(100))), 102:IExp { base: I(InstructionI(96)), power: C(2.0) }, 103:IVar("omega2"), 104:IVar("theta2"), 105:IVar("theta1"), 106:INeg(InstructionI(104)), 107:IAdd(InstructionI(105), I(InstructionI(106))), 108:IFuncCos(InstructionI(107)), 109:IVar("l2"), 110:IVar("m2"), 111:IMul(InstructionI(109), I(InstructionI(110))), 112:IExp { base: I(InstructionI(103)), power: C(2.0) }, 113:IMul(InstructionI(111), I(InstructionI(112))), 114:IExp { base: I(InstructionI(108)), power: C(2.0) }, 115:IMul(InstructionI(113), I(InstructionI(114))), 116:IVar("m1"), 117:IVar("m2"), 118:IVar("l1"), 119:IVar("omega1"), 120:IVar("theta2"), 121:IVar("theta1"), 122:INeg(InstructionI(120)), 123:IAdd(InstructionI(121), I(InstructionI(122))), 124:INeg(InstructionI(118)), 125:IExp { base: I(InstructionI(119)), power: C(2.0) }, 126:IMul(InstructionI(124), I(InstructionI(125))), 127:IFuncCos(InstructionI(123)), 128:IVar("theta1"), 129:IVar("theta2"), 130:IVar("theta1"), 131:INeg(InstructionI(129)), 132:IAdd(InstructionI(130), I(InstructionI(131))), 133:IFuncSin(InstructionI(128)), 134:IFuncSin(InstructionI(132)), 135:IMul(InstructionI(133), I(InstructionI(134))), 136:IVar("theta2"), 137:IFuncCos(InstructionI(136)), 138:IMul(InstructionI(137), C(9.81)), 139:IMul(InstructionI(126), I(InstructionI(127))), 140:INeg(InstructionI(138)), 141:IAdd(InstructionI(139), I(InstructionI(140))), 142:IMul(InstructionI(135), C(9.81)), 143:IAdd(InstructionI(116), I(InstructionI(117))), 144:IAdd(InstructionI(141), I(InstructionI(142))), 145:INeg(InstructionI(115)), 146:IMul(InstructionI(101), I(InstructionI(102))), 147:IAdd(InstructionI(145), I(InstructionI(146))), 148:IMul(InstructionI(143), I(InstructionI(144))), 149:IVar("theta2"), 150:IVar("theta1"), 151:INeg(InstructionI(149)), 152:IAdd(InstructionI(150), I(InstructionI(151))), 153:IFuncSin(InstructionI(152)), 154:IVar("m2"), 155:IExp { base: I(InstructionI(153)), power: C(2.0) }, 156:IVar("m1"), 157:IMul(InstructionI(154), I(InstructionI(155))), 158:IVar("l2"), 159:IAdd(InstructionI(156), I(InstructionI(157))), 160:IMul(InstructionI(158), I(InstructionI(159))), 161:IAdd(InstructionI(147), I(InstructionI(148))), 162:IInv(InstructionI(160)), 163:IMul(InstructionI(90), C(2.0)), 164:IMul(InstructionI(161), I(InstructionI(162))) } }), (IMul(InstructionI(53), I(InstructionI(54))), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("omega2")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 1:Expression { first: EStdFunc(EVar("omega2")), pairs: [ExprPair(EExp, EConstant(2.0)), ExprPair(EAdd, EConstant(1e-10))] }, 2:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 3:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 4:Expression { first: EConstant(1.0), pairs: [ExprPair(EMul, EStdFunc(EVar("air_resistance_coeff"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(4.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(0)))), ExprPair(EExp, EConstant(1.5)), ExprPair(ESub, EConstant(3.0)), ExprPair(EMul, EStdFunc(EVar("air_resistance_coeff"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EExp, EConstant(2.0)), ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(1)))), ExprPair(EExp, EConstant(0.5)), ExprPair(EAdd, EConstant(2.0)), ExprPair(EMul, EStdFunc(EVar("l2"))), ExprPair(EMul, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EVar("omega2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(2)))), ExprPair(EMul, EStdFunc(EFuncCos(ExpressionI(3))))] }, 5:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 6:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(5)))), ExprPair(EExp, EConstant(2.0))] }, 7:Expression { first: EStdFunc(EVar("l2")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(6))))] }, 8:Expression { first: EUnaryOp(EParentheses(ExpressionI(4))), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(7))))] } }, vals:{}, instrs:{ 0:IVar("omega2"), 1:IVar("omega2"), 2:IExp { base: I(InstructionI(1)), power: C(2.0) }, 3:IAdd(InstructionI(2), C(1e-10)), 4:IExp { base: I(InstructionI(3)), power: C(1.5) }, 5:IVar("air_resistance_coeff"), 6:IInv(InstructionI(4)), 7:IMul(InstructionI(5), I(InstructionI(6))), 8:IExp { base: I(InstructionI(0)), power: C(4.0) }, 9:IVar("omega2"), 10:IVar("omega2"), 11:IExp { base: I(InstructionI(10)), power: C(2.0) }, 12:IAdd(InstructionI(11), C(1e-10)), 13:IExp { base: I(InstructionI(12)), power: C(0.5) }, 14:IVar("air_resistance_coeff"), 15:IInv(InstructionI(13)), 16:IMul(InstructionI(14), I(InstructionI(15))), 17:IExp { base: I(InstructionI(9)), power: C(2.0) }, 18:IMul(InstructionI(16), I(InstructionI(17))), 19:IMul(InstructionI(18), C(3.0)), 20:IVar("theta2"), 21:IVar("theta1"), 22:INeg(InstructionI(20)), 23:IAdd(InstructionI(21), I(InstructionI(22))), 24:IVar("theta2"), 25:IVar("theta1"), 26:INeg(InstructionI(24)), 27:IAdd(InstructionI(25), I(InstructionI(26))), 28:IVar("l2"), 29:IVar("m2"), 30:IMul(InstructionI(28), I(InstructionI(29))), 31:IVar("omega2"), 32:IMul(InstructionI(30), I(InstructionI(31))), 33:IFuncSin(InstructionI(23)), 34:IMul(InstructionI(32), I(InstructionI(33))), 35:IFuncCos(InstructionI(27)), 36:IMul(InstructionI(34), I(InstructionI(35))), 37:INeg(InstructionI(19)), 38:IMul(InstructionI(7), I(InstructionI(8))), 39:IAdd(InstructionI(37), I(InstructionI(38))), 40:IMul(InstructionI(36), C(2.0)), 41:IVar("theta2"), 42:IVar("theta1"), 43:INeg(InstructionI(41)), 44:IAdd(InstructionI(42), I(InstructionI(43))), 45:IFuncSin(InstructionI(44)), 46:IVar("m2"), 47:IExp { base: I(InstructionI(45)), power: C(2.0) }, 48:IVar("m1"), 49:IMul(InstructionI(46), I(InstructionI(47))), 50:IVar("l2"), 51:IAdd(InstructionI(48), I(InstructionI(49))), 52:IMul(InstructionI(50), I(InstructionI(51))), 53:IAdd(InstructionI(39), I(InstructionI(40))), 54:IInv(InstructionI(52)) } })]]*/

pub fn eval_dfdx(params: &[f64]) -> DMatrix<f64> {
    let mut v: Vec<Vec<f64>> = vec![vec![0.0; 4]; 4];
    let t0 = 0.00000000000000000e0;
    v[0][0] = t0;
    let t1 = 1.00000000000000000e0;
    v[0][1] = t1;
    let t2 = 0.00000000000000000e0;
    v[0][2] = t2;
    let t3 = 0.00000000000000000e0;
    v[0][3] = t3;
    let t4 = params[10];
    let t5 = params[1];
    let t6 = params[1];
    let t7 = t6.powf(2.00000000000000000e0);
    let t8 = t7 + 1.00000000000000004e-10;
    let t9 = t8.powf(5.00000000000000000e-1);
    let t10 = -t4;
    let t11 = 1.0 / t9;
    let t12 = t10 * t11;
    let t13 = t5.powf(3.00000000000000000e0);
    let t14 = params[1];
    let t15 = params[2];
    let t16 = params[0];
    let t17 = -t15;
    let t18 = t16 + t17;
    let t19 = params[8];
    let t20 = t14.powf(2.00000000000000000e0);
    let t21 = t19 * t20;
    let t22 = t18.cos();
    let t23 = params[3];
    let t24 = params[9];
    let t25 = t23.powf(2.00000000000000000e0);
    let t26 = t21 * t22;
    let t27 = t24 * t25;
    let t28 = params[2];
    let t29 = params[0];
    let t30 = -t28;
    let t31 = t29 + t30;
    let t32 = params[7];
    let t33 = t26 + t27;
    let t34 = t32 * t33;
    let t35 = t31.sin();
    let t36 = t34 * t35;
    let t37 = params[2];
    let t38 = params[2];
    let t39 = params[0];
    let t40 = -t38;
    let t41 = t39 + t40;
    let t42 = params[7];
    let t43 = t37.sin();
    let t44 = t42 * t43;
    let t45 = t41.cos();
    let t46 = t44 * t45;
    let t47 = params[6];
    let t48 = params[7];
    let t49 = t47 * 9.81000000000000050e0;
    let t50 = t48 * 9.81000000000000050e0;
    let t51 = params[0];
    let t52 = t49 + t50;
    let t53 = t51.sin();
    let t54 = t52 * t53;
    let t55 = -t36;
    let t56 = t12 * t13;
    let t57 = t55 + t56;
    let t58 = t46 * 9.81000000000000050e0;
    let t59 = t57 + t58;
    let t60 = -t54;
    let t61 = t59 + t60;
    let t62 = params[4];
    let t63 = params[2];
    let t64 = params[0];
    let t65 = -t63;
    let t66 = t64 + t65;
    let t67 = params[2];
    let t68 = params[0];
    let t69 = -t67;
    let t70 = t68 + t69;
    let t71 = params[2];
    let t72 = params[0];
    let t73 = -t71;
    let t74 = t72 + t73;
    let t75 = t74.sin();
    let t76 = params[7];
    let t77 = t75.powf(2.00000000000000000e0);
    let t78 = params[6];
    let t79 = t76 * t77;
    let t80 = t78 + t79;
    let t81 = params[8];
    let t82 = t80.powf(2.00000000000000000e0);
    let t83 = t81 * t82;
    let t84 = params[7];
    let t85 = t61 + t62;
    let t86 = t84 * t85;
    let t87 = t66.sin();
    let t88 = t86 * t87;
    let t89 = 1.0 / t83;
    let t90 = t88 * t89;
    let t91 = t70.cos();
    let t92 = t90 * t91;
    let t93 = params[1];
    let t94 = params[2];
    let t95 = params[0];
    let t96 = -t94;
    let t97 = t95 + t96;
    let t98 = t97.sin();
    let t99 = params[8];
    let t100 = params[7];
    let t101 = t99 * t100;
    let t102 = t93.powf(2.00000000000000000e0);
    let t103 = t101 * t102;
    let t104 = t98.powf(2.00000000000000000e0);
    let t105 = params[1];
    let t106 = params[2];
    let t107 = params[0];
    let t108 = -t106;
    let t109 = t107 + t108;
    let t110 = params[8];
    let t111 = t105.powf(2.00000000000000000e0);
    let t112 = t110 * t111;
    let t113 = t109.cos();
    let t114 = params[3];
    let t115 = params[9];
    let t116 = t114.powf(2.00000000000000000e0);
    let t117 = t112 * t113;
    let t118 = t115 * t116;
    let t119 = params[2];
    let t120 = params[0];
    let t121 = -t119;
    let t122 = t120 + t121;
    let t123 = params[7];
    let t124 = t117 + t118;
    let t125 = t123 * t124;
    let t126 = t122.cos();
    let t127 = t125 * t126;
    let t128 = params[2];
    let t129 = params[2];
    let t130 = params[0];
    let t131 = -t129;
    let t132 = t130 + t131;
    let t133 = params[7];
    let t134 = t128.sin();
    let t135 = t133 * t134;
    let t136 = t132.sin();
    let t137 = t135 * t136;
    let t138 = t137 * 9.81000000000000050e0;
    let t139 = params[6];
    let t140 = params[7];
    let t141 = t139 * 9.81000000000000050e0;
    let t142 = t140 * 9.81000000000000050e0;
    let t143 = params[0];
    let t144 = t141 + t142;
    let t145 = t143.cos();
    let t146 = t144 * t145;
    let t147 = t103 * t104;
    let t148 = -t127;
    let t149 = t147 + t148;
    let t150 = -t138;
    let t151 = t149 + t150;
    let t152 = -t146;
    let t153 = params[2];
    let t154 = params[0];
    let t155 = -t153;
    let t156 = t154 + t155;
    let t157 = t156.sin();
    let t158 = params[7];
    let t159 = t157.powf(2.00000000000000000e0);
    let t160 = params[6];
    let t161 = t158 * t159;
    let t162 = params[8];
    let t163 = t160 + t161;
    let t164 = t162 * t163;
    let t165 = t151 + t152;
    let t166 = 1.0 / t164;
    let t167 = t92 * -2.00000000000000000e0;
    let t168 = t165 * t166;
    let t169 = t167 + t168;
    v[1][0] = t169;
    let t170 = params[1];
    let t171 = params[1];
    let t172 = t171.powf(2.00000000000000000e0);
    let t173 = t172 + 1.00000000000000004e-10;
    let t174 = t173.powf(1.50000000000000000e0);
    let t175 = params[10];
    let t176 = 1.0 / t174;
    let t177 = t175 * t176;
    let t178 = t170.powf(4.00000000000000000e0);
    let t179 = params[1];
    let t180 = params[1];
    let t181 = t180.powf(2.00000000000000000e0);
    let t182 = t181 + 1.00000000000000004e-10;
    let t183 = t182.powf(5.00000000000000000e-1);
    let t184 = params[10];
    let t185 = 1.0 / t183;
    let t186 = t184 * t185;
    let t187 = t179.powf(2.00000000000000000e0);
    let t188 = t186 * t187;
    let t189 = t188 * 3.00000000000000000e0;
    let t190 = params[2];
    let t191 = params[0];
    let t192 = -t190;
    let t193 = t191 + t192;
    let t194 = params[2];
    let t195 = params[0];
    let t196 = -t194;
    let t197 = t195 + t196;
    let t198 = params[8];
    let t199 = params[7];
    let t200 = t198 * t199;
    let t201 = params[1];
    let t202 = t200 * t201;
    let t203 = t193.sin();
    let t204 = t202 * t203;
    let t205 = t197.cos();
    let t206 = t204 * t205;
    let t207 = t206 * 2.00000000000000000e0;
    let t208 = t177 * t178;
    let t209 = -t189;
    let t210 = t208 + t209;
    let t211 = -t207;
    let t212 = params[2];
    let t213 = params[0];
    let t214 = -t212;
    let t215 = t213 + t214;
    let t216 = t215.sin();
    let t217 = params[7];
    let t218 = t216.powf(2.00000000000000000e0);
    let t219 = params[6];
    let t220 = t217 * t218;
    let t221 = params[8];
    let t222 = t219 + t220;
    let t223 = t221 * t222;
    let t224 = t210 + t211;
    let t225 = 1.0 / t223;
    let t226 = t224 * t225;
    v[1][1] = t226;
    let t227 = params[10];
    let t228 = params[1];
    let t229 = params[1];
    let t230 = t229.powf(2.00000000000000000e0);
    let t231 = t230 + 1.00000000000000004e-10;
    let t232 = t231.powf(5.00000000000000000e-1);
    let t233 = -t227;
    let t234 = 1.0 / t232;
    let t235 = t233 * t234;
    let t236 = t228.powf(3.00000000000000000e0);
    let t237 = params[1];
    let t238 = params[2];
    let t239 = params[0];
    let t240 = -t238;
    let t241 = t239 + t240;
    let t242 = params[8];
    let t243 = t237.powf(2.00000000000000000e0);
    let t244 = t242 * t243;
    let t245 = t241.cos();
    let t246 = params[3];
    let t247 = params[9];
    let t248 = t246.powf(2.00000000000000000e0);
    let t249 = t244 * t245;
    let t250 = t247 * t248;
    let t251 = params[2];
    let t252 = params[0];
    let t253 = -t251;
    let t254 = t252 + t253;
    let t255 = params[7];
    let t256 = t249 + t250;
    let t257 = t255 * t256;
    let t258 = t254.sin();
    let t259 = t257 * t258;
    let t260 = params[2];
    let t261 = params[2];
    let t262 = params[0];
    let t263 = -t261;
    let t264 = t262 + t263;
    let t265 = params[7];
    let t266 = t260.sin();
    let t267 = t265 * t266;
    let t268 = t264.cos();
    let t269 = t267 * t268;
    let t270 = params[6];
    let t271 = params[7];
    let t272 = t270 * 9.81000000000000050e0;
    let t273 = t271 * 9.81000000000000050e0;
    let t274 = params[0];
    let t275 = t272 + t273;
    let t276 = t274.sin();
    let t277 = t275 * t276;
    let t278 = -t259;
    let t279 = t235 * t236;
    let t280 = t278 + t279;
    let t281 = t269 * 9.81000000000000050e0;
    let t282 = t280 + t281;
    let t283 = -t277;
    let t284 = t282 + t283;
    let t285 = params[4];
    let t286 = params[2];
    let t287 = params[0];
    let t288 = -t286;
    let t289 = t287 + t288;
    let t290 = params[2];
    let t291 = params[0];
    let t292 = -t290;
    let t293 = t291 + t292;
    let t294 = params[2];
    let t295 = params[0];
    let t296 = -t294;
    let t297 = t295 + t296;
    let t298 = t297.sin();
    let t299 = params[7];
    let t300 = t298.powf(2.00000000000000000e0);
    let t301 = params[6];
    let t302 = t299 * t300;
    let t303 = t301 + t302;
    let t304 = params[8];
    let t305 = t303.powf(2.00000000000000000e0);
    let t306 = t304 * t305;
    let t307 = params[7];
    let t308 = t284 + t285;
    let t309 = t307 * t308;
    let t310 = t289.sin();
    let t311 = t309 * t310;
    let t312 = 1.0 / t306;
    let t313 = t311 * t312;
    let t314 = t293.cos();
    let t315 = t313 * t314;
    let t316 = params[8];
    let t317 = params[1];
    let t318 = params[2];
    let t319 = params[0];
    let t320 = -t318;
    let t321 = t319 + t320;
    let t322 = t321.sin();
    let t323 = -t316;
    let t324 = params[7];
    let t325 = t323 * t324;
    let t326 = t317.powf(2.00000000000000000e0);
    let t327 = t325 * t326;
    let t328 = t322.powf(2.00000000000000000e0);
    let t329 = params[1];
    let t330 = params[2];
    let t331 = params[0];
    let t332 = -t330;
    let t333 = t331 + t332;
    let t334 = params[8];
    let t335 = t329.powf(2.00000000000000000e0);
    let t336 = t334 * t335;
    let t337 = t333.cos();
    let t338 = params[3];
    let t339 = params[9];
    let t340 = t338.powf(2.00000000000000000e0);
    let t341 = t336 * t337;
    let t342 = t339 * t340;
    let t343 = params[2];
    let t344 = params[0];
    let t345 = -t343;
    let t346 = t344 + t345;
    let t347 = params[7];
    let t348 = t341 + t342;
    let t349 = t347 * t348;
    let t350 = t346.cos();
    let t351 = params[2];
    let t352 = params[2];
    let t353 = params[0];
    let t354 = -t352;
    let t355 = t353 + t354;
    let t356 = params[7];
    let t357 = t351.sin();
    let t358 = t356 * t357;
    let t359 = t355.sin();
    let t360 = t358 * t359;
    let t361 = params[2];
    let t362 = params[2];
    let t363 = params[0];
    let t364 = -t362;
    let t365 = t363 + t364;
    let t366 = params[7];
    let t367 = t361.cos();
    let t368 = t366 * t367;
    let t369 = t365.cos();
    let t370 = t368 * t369;
    let t371 = t327 * t328;
    let t372 = t349 * t350;
    let t373 = t371 + t372;
    let t374 = t360 * 9.81000000000000050e0;
    let t375 = t373 + t374;
    let t376 = t370 * 9.81000000000000050e0;
    let t377 = params[2];
    let t378 = params[0];
    let t379 = -t377;
    let t380 = t378 + t379;
    let t381 = t380.sin();
    let t382 = params[7];
    let t383 = t381.powf(2.00000000000000000e0);
    let t384 = params[6];
    let t385 = t382 * t383;
    let t386 = params[8];
    let t387 = t384 + t385;
    let t388 = t386 * t387;
    let t389 = t375 + t376;
    let t390 = 1.0 / t388;
    let t391 = t315 * 2.00000000000000000e0;
    let t392 = t389 * t390;
    let t393 = t391 + t392;
    v[1][2] = t393;
    let t394 = params[2];
    let t395 = params[0];
    let t396 = -t394;
    let t397 = t395 + t396;
    let t398 = params[2];
    let t399 = params[0];
    let t400 = -t398;
    let t401 = t399 + t400;
    let t402 = t401.sin();
    let t403 = params[7];
    let t404 = t402.powf(2.00000000000000000e0);
    let t405 = params[6];
    let t406 = t403 * t404;
    let t407 = params[8];
    let t408 = t405 + t406;
    let t409 = t407 * t408;
    let t410 = params[9];
    let t411 = params[7];
    let t412 = t410 * t411;
    let t413 = params[3];
    let t414 = t412 * t413;
    let t415 = 1.0 / t409;
    let t416 = t414 * t415;
    let t417 = t397.sin();
    let t418 = t416 * t417;
    let t419 = t418 * -2.00000000000000000e0;
    v[1][3] = t419;
    let t420 = 0.00000000000000000e0;
    v[2][0] = t420;
    let t421 = 0.00000000000000000e0;
    v[2][1] = t421;
    let t422 = 0.00000000000000000e0;
    v[2][2] = t422;
    let t423 = 1.00000000000000000e0;
    v[2][3] = t423;
    let t424 = params[10];
    let t425 = params[3];
    let t426 = params[3];
    let t427 = t426.powf(2.00000000000000000e0);
    let t428 = t427 + 1.00000000000000004e-10;
    let t429 = t428.powf(5.00000000000000000e-1);
    let t430 = -t424;
    let t431 = 1.0 / t429;
    let t432 = t430 * t431;
    let t433 = t425.powf(3.00000000000000000e0);
    let t434 = params[3];
    let t435 = params[2];
    let t436 = params[0];
    let t437 = -t435;
    let t438 = t436 + t437;
    let t439 = params[2];
    let t440 = params[0];
    let t441 = -t439;
    let t442 = t440 + t441;
    let t443 = params[9];
    let t444 = params[7];
    let t445 = t443 * t444;
    let t446 = t434.powf(2.00000000000000000e0);
    let t447 = t445 * t446;
    let t448 = t438.sin();
    let t449 = t447 * t448;
    let t450 = t442.cos();
    let t451 = params[6];
    let t452 = params[7];
    let t453 = params[1];
    let t454 = params[2];
    let t455 = params[0];
    let t456 = -t454;
    let t457 = t455 + t456;
    let t458 = params[8];
    let t459 = t453.powf(2.00000000000000000e0);
    let t460 = t458 * t459;
    let t461 = t457.sin();
    let t462 = params[0];
    let t463 = params[2];
    let t464 = params[0];
    let t465 = -t463;
    let t466 = t464 + t465;
    let t467 = t462.sin();
    let t468 = t466.cos();
    let t469 = t467 * t468;
    let t470 = params[2];
    let t471 = t470.sin();
    let t472 = t471 * 9.81000000000000050e0;
    let t473 = t460 * t461;
    let t474 = -t472;
    let t475 = t473 + t474;
    let t476 = t469 * 9.81000000000000050e0;
    let t477 = t451 + t452;
    let t478 = t475 + t476;
    let t479 = t432 * t433;
    let t480 = t449 * t450;
    let t481 = t479 + t480;
    let t482 = params[5];
    let t483 = t481 + t482;
    let t484 = t477 * t478;
    let t485 = params[2];
    let t486 = params[0];
    let t487 = -t485;
    let t488 = t486 + t487;
    let t489 = params[2];
    let t490 = params[0];
    let t491 = -t489;
    let t492 = t490 + t491;
    let t493 = params[2];
    let t494 = params[0];
    let t495 = -t493;
    let t496 = t494 + t495;
    let t497 = t496.sin();
    let t498 = params[7];
    let t499 = t497.powf(2.00000000000000000e0);
    let t500 = params[6];
    let t501 = t498 * t499;
    let t502 = t500 + t501;
    let t503 = params[9];
    let t504 = t502.powf(2.00000000000000000e0);
    let t505 = t503 * t504;
    let t506 = params[7];
    let t507 = t483 + t484;
    let t508 = t506 * t507;
    let t509 = t488.sin();
    let t510 = t508 * t509;
    let t511 = 1.0 / t505;
    let t512 = t510 * t511;
    let t513 = t492.cos();
    let t514 = t512 * t513;
    let t515 = params[9];
    let t516 = params[3];
    let t517 = params[2];
    let t518 = params[0];
    let t519 = -t517;
    let t520 = t518 + t519;
    let t521 = t520.sin();
    let t522 = -t515;
    let t523 = params[7];
    let t524 = t522 * t523;
    let t525 = t516.powf(2.00000000000000000e0);
    let t526 = t524 * t525;
    let t527 = t521.powf(2.00000000000000000e0);
    let t528 = params[3];
    let t529 = params[2];
    let t530 = params[0];
    let t531 = -t529;
    let t532 = t530 + t531;
    let t533 = t532.cos();
    let t534 = params[9];
    let t535 = params[7];
    let t536 = t534 * t535;
    let t537 = t528.powf(2.00000000000000000e0);
    let t538 = t536 * t537;
    let t539 = t533.powf(2.00000000000000000e0);
    let t540 = params[6];
    let t541 = params[7];
    let t542 = params[1];
    let t543 = params[2];
    let t544 = params[0];
    let t545 = -t543;
    let t546 = t544 + t545;
    let t547 = params[8];
    let t548 = t542.powf(2.00000000000000000e0);
    let t549 = t547 * t548;
    let t550 = t546.cos();
    let t551 = params[0];
    let t552 = params[2];
    let t553 = params[0];
    let t554 = -t552;
    let t555 = t553 + t554;
    let t556 = t551.sin();
    let t557 = t555.sin();
    let t558 = t556 * t557;
    let t559 = t558 * 9.81000000000000050e0;
    let t560 = params[0];
    let t561 = params[2];
    let t562 = params[0];
    let t563 = -t561;
    let t564 = t562 + t563;
    let t565 = t560.cos();
    let t566 = t564.cos();
    let t567 = t565 * t566;
    let t568 = -t559;
    let t569 = t549 * t550;
    let t570 = t568 + t569;
    let t571 = t567 * 9.81000000000000050e0;
    let t572 = t540 + t541;
    let t573 = t570 + t571;
    let t574 = t526 * t527;
    let t575 = t538 * t539;
    let t576 = t574 + t575;
    let t577 = t572 * t573;
    let t578 = params[2];
    let t579 = params[0];
    let t580 = -t578;
    let t581 = t579 + t580;
    let t582 = t581.sin();
    let t583 = params[7];
    let t584 = t582.powf(2.00000000000000000e0);
    let t585 = params[6];
    let t586 = t583 * t584;
    let t587 = params[9];
    let t588 = t585 + t586;
    let t589 = t587 * t588;
    let t590 = t576 + t577;
    let t591 = 1.0 / t589;
    let t592 = t514 * -2.00000000000000000e0;
    let t593 = t590 * t591;
    let t594 = t592 + t593;
    v[3][0] = t594;
    let t595 = params[6];
    let t596 = params[7];
    let t597 = params[2];
    let t598 = params[0];
    let t599 = -t597;
    let t600 = t598 + t599;
    let t601 = params[2];
    let t602 = params[0];
    let t603 = -t601;
    let t604 = t602 + t603;
    let t605 = t604.sin();
    let t606 = params[7];
    let t607 = t605.powf(2.00000000000000000e0);
    let t608 = params[6];
    let t609 = t606 * t607;
    let t610 = params[9];
    let t611 = t608 + t609;
    let t612 = t610 * t611;
    let t613 = params[8];
    let t614 = params[1];
    let t615 = t613 * t614;
    let t616 = t595 + t596;
    let t617 = t615 * t616;
    let t618 = 1.0 / t612;
    let t619 = t617 * t618;
    let t620 = t600.sin();
    let t621 = t619 * t620;
    let t622 = t621 * 2.00000000000000000e0;
    v[3][1] = t622;
    let t623 = params[10];
    let t624 = params[3];
    let t625 = params[3];
    let t626 = t625.powf(2.00000000000000000e0);
    let t627 = t626 + 1.00000000000000004e-10;
    let t628 = t627.powf(5.00000000000000000e-1);
    let t629 = -t623;
    let t630 = 1.0 / t628;
    let t631 = t629 * t630;
    let t632 = t624.powf(3.00000000000000000e0);
    let t633 = params[3];
    let t634 = params[2];
    let t635 = params[0];
    let t636 = -t634;
    let t637 = t635 + t636;
    let t638 = params[2];
    let t639 = params[0];
    let t640 = -t638;
    let t641 = t639 + t640;
    let t642 = params[9];
    let t643 = params[7];
    let t644 = t642 * t643;
    let t645 = t633.powf(2.00000000000000000e0);
    let t646 = t644 * t645;
    let t647 = t637.sin();
    let t648 = t646 * t647;
    let t649 = t641.cos();
    let t650 = params[6];
    let t651 = params[7];
    let t652 = params[1];
    let t653 = params[2];
    let t654 = params[0];
    let t655 = -t653;
    let t656 = t654 + t655;
    let t657 = params[8];
    let t658 = t652.powf(2.00000000000000000e0);
    let t659 = t657 * t658;
    let t660 = t656.sin();
    let t661 = params[0];
    let t662 = params[2];
    let t663 = params[0];
    let t664 = -t662;
    let t665 = t663 + t664;
    let t666 = t661.sin();
    let t667 = t665.cos();
    let t668 = t666 * t667;
    let t669 = params[2];
    let t670 = t669.sin();
    let t671 = t670 * 9.81000000000000050e0;
    let t672 = t659 * t660;
    let t673 = -t671;
    let t674 = t672 + t673;
    let t675 = t668 * 9.81000000000000050e0;
    let t676 = t650 + t651;
    let t677 = t674 + t675;
    let t678 = t631 * t632;
    let t679 = t648 * t649;
    let t680 = t678 + t679;
    let t681 = params[5];
    let t682 = t680 + t681;
    let t683 = t676 * t677;
    let t684 = params[2];
    let t685 = params[0];
    let t686 = -t684;
    let t687 = t685 + t686;
    let t688 = params[2];
    let t689 = params[0];
    let t690 = -t688;
    let t691 = t689 + t690;
    let t692 = params[2];
    let t693 = params[0];
    let t694 = -t692;
    let t695 = t693 + t694;
    let t696 = t695.sin();
    let t697 = params[7];
    let t698 = t696.powf(2.00000000000000000e0);
    let t699 = params[6];
    let t700 = t697 * t698;
    let t701 = t699 + t700;
    let t702 = params[9];
    let t703 = t701.powf(2.00000000000000000e0);
    let t704 = t702 * t703;
    let t705 = params[7];
    let t706 = t682 + t683;
    let t707 = t705 * t706;
    let t708 = t687.sin();
    let t709 = t707 * t708;
    let t710 = 1.0 / t704;
    let t711 = t709 * t710;
    let t712 = t691.cos();
    let t713 = t711 * t712;
    let t714 = params[3];
    let t715 = params[2];
    let t716 = params[0];
    let t717 = -t715;
    let t718 = t716 + t717;
    let t719 = t718.sin();
    let t720 = params[9];
    let t721 = params[7];
    let t722 = t720 * t721;
    let t723 = t714.powf(2.00000000000000000e0);
    let t724 = t722 * t723;
    let t725 = t719.powf(2.00000000000000000e0);
    let t726 = params[3];
    let t727 = params[2];
    let t728 = params[0];
    let t729 = -t727;
    let t730 = t728 + t729;
    let t731 = t730.cos();
    let t732 = params[9];
    let t733 = params[7];
    let t734 = t732 * t733;
    let t735 = t726.powf(2.00000000000000000e0);
    let t736 = t734 * t735;
    let t737 = t731.powf(2.00000000000000000e0);
    let t738 = t736 * t737;
    let t739 = params[6];
    let t740 = params[7];
    let t741 = params[8];
    let t742 = params[1];
    let t743 = params[2];
    let t744 = params[0];
    let t745 = -t743;
    let t746 = t744 + t745;
    let t747 = -t741;
    let t748 = t742.powf(2.00000000000000000e0);
    let t749 = t747 * t748;
    let t750 = t746.cos();
    let t751 = params[0];
    let t752 = params[2];
    let t753 = params[0];
    let t754 = -t752;
    let t755 = t753 + t754;
    let t756 = t751.sin();
    let t757 = t755.sin();
    let t758 = t756 * t757;
    let t759 = params[2];
    let t760 = t759.cos();
    let t761 = t760 * 9.81000000000000050e0;
    let t762 = t749 * t750;
    let t763 = -t761;
    let t764 = t762 + t763;
    let t765 = t758 * 9.81000000000000050e0;
    let t766 = t739 + t740;
    let t767 = t764 + t765;
    let t768 = -t738;
    let t769 = t724 * t725;
    let t770 = t768 + t769;
    let t771 = t766 * t767;
    let t772 = params[2];
    let t773 = params[0];
    let t774 = -t772;
    let t775 = t773 + t774;
    let t776 = t775.sin();
    let t777 = params[7];
    let t778 = t776.powf(2.00000000000000000e0);
    let t779 = params[6];
    let t780 = t777 * t778;
    let t781 = params[9];
    let t782 = t779 + t780;
    let t783 = t781 * t782;
    let t784 = t770 + t771;
    let t785 = 1.0 / t783;
    let t786 = t713 * 2.00000000000000000e0;
    let t787 = t784 * t785;
    let t788 = t786 + t787;
    v[3][2] = t788;
    let t789 = params[3];
    let t790 = params[3];
    let t791 = t790.powf(2.00000000000000000e0);
    let t792 = t791 + 1.00000000000000004e-10;
    let t793 = t792.powf(1.50000000000000000e0);
    let t794 = params[10];
    let t795 = 1.0 / t793;
    let t796 = t794 * t795;
    let t797 = t789.powf(4.00000000000000000e0);
    let t798 = params[3];
    let t799 = params[3];
    let t800 = t799.powf(2.00000000000000000e0);
    let t801 = t800 + 1.00000000000000004e-10;
    let t802 = t801.powf(5.00000000000000000e-1);
    let t803 = params[10];
    let t804 = 1.0 / t802;
    let t805 = t803 * t804;
    let t806 = t798.powf(2.00000000000000000e0);
    let t807 = t805 * t806;
    let t808 = t807 * 3.00000000000000000e0;
    let t809 = params[2];
    let t810 = params[0];
    let t811 = -t809;
    let t812 = t810 + t811;
    let t813 = params[2];
    let t814 = params[0];
    let t815 = -t813;
    let t816 = t814 + t815;
    let t817 = params[9];
    let t818 = params[7];
    let t819 = t817 * t818;
    let t820 = params[3];
    let t821 = t819 * t820;
    let t822 = t812.sin();
    let t823 = t821 * t822;
    let t824 = t816.cos();
    let t825 = t823 * t824;
    let t826 = -t808;
    let t827 = t796 * t797;
    let t828 = t826 + t827;
    let t829 = t825 * 2.00000000000000000e0;
    let t830 = params[2];
    let t831 = params[0];
    let t832 = -t830;
    let t833 = t831 + t832;
    let t834 = t833.sin();
    let t835 = params[7];
    let t836 = t834.powf(2.00000000000000000e0);
    let t837 = params[6];
    let t838 = t835 * t836;
    let t839 = params[9];
    let t840 = t837 + t838;
    let t841 = t839 * t840;
    let t842 = t828 + t829;
    let t843 = 1.0 / t841;
    let t844 = t842 * t843;
    v[3][3] = t844;
    vec_to_dmat(&v)
}
/*
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IFuncSin(InstructionI(3)), 5:IVar("m2"), 6:IExp { base: I(InstructionI(4)), power: C(2.0) }, 7:IVar("m1"), 8:IMul(InstructionI(5), I(InstructionI(6))), 9:IVar("l1"), 10:IAdd(InstructionI(7), I(InstructionI(8))), 11:IMul(InstructionI(9), I(InstructionI(10))) } }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{} }
CompileSlab{ instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IFuncSin(InstructionI(3)), 5:IVar("m2"), 6:IExp { base: I(InstructionI(4)), power: C(2.0) }, 7:IVar("m1"), 8:IMul(InstructionI(5), I(InstructionI(6))), 9:IVar("l2"), 10:IAdd(InstructionI(7), I(InstructionI(8))), 11:IMul(InstructionI(9), I(InstructionI(10))) } }



  [[(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} })], [(IInv(InstructionI(11)), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(0)))), ExprPair(EExp, EConstant(2.0))] }, 2:Expression { first: EStdFunc(EVar("l1")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(1))))] }, 3:Expression { first: EConstant(1.0), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(2))))] } }, vals:{}, instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IFuncSin(InstructionI(3)), 5:IVar("m2"), 6:IExp { base: I(InstructionI(4)), power: C(2.0) }, 7:IVar("m1"), 8:IMul(InstructionI(5), I(InstructionI(6))), 9:IVar("l1"), 10:IAdd(InstructionI(7), I(InstructionI(8))), 11:IMul(InstructionI(9), I(InstructionI(10))) } }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} })], [(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} })], [(IConst(0.0), Slab{ exprs:{ 0:Expression { first: EConstant(0.0), pairs: [] } }, vals:{}, instrs:{} }), (IInv(InstructionI(11)), Slab{ exprs:{ 0:Expression { first: EStdFunc(EVar("theta1")), pairs: [ExprPair(ESub, EStdFunc(EVar("theta2")))] }, 1:Expression { first: EStdFunc(EVar("m1")), pairs: [ExprPair(EAdd, EStdFunc(EVar("m2"))), ExprPair(EMul, EStdFunc(EFuncSin(ExpressionI(0)))), ExprPair(EExp, EConstant(2.0))] }, 2:Expression { first: EStdFunc(EVar("l2")), pairs: [ExprPair(EMul, EUnaryOp(EParentheses(ExpressionI(1))))] }, 3:Expression { first: EConstant(1.0), pairs: [ExprPair(EDiv, EUnaryOp(EParentheses(ExpressionI(2))))] } }, vals:{}, instrs:{ 0:IVar("theta2"), 1:IVar("theta1"), 2:INeg(InstructionI(0)), 3:IAdd(InstructionI(1), I(InstructionI(2))), 4:IFuncSin(InstructionI(3)), 5:IVar("m2"), 6:IExp { base: I(InstructionI(4)), power: C(2.0) }, 7:IVar("m1"), 8:IMul(InstructionI(5), I(InstructionI(6))), 9:IVar("l2"), 10:IAdd(InstructionI(7), I(InstructionI(8))), 11:IMul(InstructionI(9), I(InstructionI(10))) } })]]*/

pub fn eval_dfdu(params: &[f64]) -> DMatrix<f64> {
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
    let t7 = params[7];
    let t8 = t6.powf(2.00000000000000000e0);
    let t9 = params[6];
    let t10 = t7 * t8;
    let t11 = params[8];
    let t12 = t9 + t10;
    let t13 = t11 * t12;
    let t14 = 1.0 / t13;
    v[1][0] = t14;
    let t15 = 0.00000000000000000e0;
    v[1][1] = t15;
    let t16 = 0.00000000000000000e0;
    v[2][0] = t16;
    let t17 = 0.00000000000000000e0;
    v[2][1] = t17;
    let t18 = 0.00000000000000000e0;
    v[3][0] = t18;
    let t19 = params[2];
    let t20 = params[0];
    let t21 = -t19;
    let t22 = t20 + t21;
    let t23 = t22.sin();
    let t24 = params[7];
    let t25 = t23.powf(2.00000000000000000e0);
    let t26 = params[6];
    let t27 = t24 * t25;
    let t28 = params[9];
    let t29 = t26 + t27;
    let t30 = t28 * t29;
    let t31 = 1.0 / t30;
    v[3][1] = t31;
    vec_to_dmat(&v)
}

use std::f64::consts::PI;
use std::sync::Arc;

use control_rs::numeric_services::symbolic::{
    ExprRegistry, SymbolicExpr, SymbolicFunction, TryIntoEvalResult,
};
use control_rs::physics::constants as c;
use control_rs::physics::discretizer::rk4_numeric::RK4Numeric;
use control_rs::physics::discretizer::{
    NumericDiscretizer, NumericFunction, RK4Symbolic, SymbolicDiscretizer,
};
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

    let model = DoublePendulum::new(m1, m2, l1, l2, air_resistance_coeff, Some(&registry), false);
    let state_0 = DoublePendulumState::new(theta1, omega1, theta2, omega2);
    let state_symbol = registry.get_vector(c::STATE_SYMBOLIC).unwrap();
    let input_symbol = registry.get_vector(c::INPUT_SYMBOLIC).unwrap();
    let jacobian_symbols = state_symbol.extend(&input_symbol);
    let symbolic_f = model.dynamics_symbolic(&state_symbol, &registry);
    //model.store_params(&registry);

    let params_state = DVector::from_vec(vec![theta1, omega1, theta2, omega2]);
    let params_input = DVector::from_vec(vec![0.0, 0.0]);
    let params_params = DVector::from_vec(vec![m1, m2, l1, l2, air_resistance_coeff]);
    let mut params = params_state.as_slice().to_vec();
    params.extend(params_input.as_slice());
    params.extend(params_params.as_slice());

    /*
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
    */

    /////
    let discretizer = RK4Symbolic::new(&model, Arc::clone(&registry)).unwrap();
    registry.insert_var(c::TIME_DELTA_SYMBOLIC, dt);
    let fa = discretizer.jacobian_x().unwrap();
    let fb = discretizer.jacobian_u().unwrap();
    let start = Instant::now();
    /*
    let a = fa.evaluate(&params).unwrap();
    let b = fb.evaluate(&params).unwrap();
    */
    let duration_symbolic = start.elapsed();
    let original_params = params.clone();

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
    params.extend(params_params.as_slice());
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
    params.extend(params_params.as_slice());
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
    params.extend(params_params.as_slice());
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

    /*
    println!("{}, {:?}", symbolic_f_dx, df_dx);
    println!("{}, {:?}", symbolic_f_du, df_du);
    println!("{}, {:?}", symbolic_eval_f, f);
    println!("{}, {}", dx_next_dx, a);
    println!("{}, {}", dx_next_du, b);
    */

    println!("{:?}, {:?}", duration_symbolic, duration_numeric);

    //
    let dfdx = NumericFunction(Arc::new(move |vals: &[f64]| eval_dfdx(vals)));
    let dfdu = NumericFunction(Arc::new(move |vals: &[f64]| eval_dfdu(vals)));
    let discretizer = RK4Numeric::new(Arc::new(model.clone()), dfdx, dfdu, dt).unwrap();
    let j1 = discretizer.jacobian_x(&original_params);
    let j2 = discretizer.jacobian_u(&original_params);
    println!("{}, {}", j1, j2);
}
