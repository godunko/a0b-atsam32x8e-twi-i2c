--
--  Copyright (C) 2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  pragma Restrictions (No_Elaboration_Code);

package A0B.I2C.ATSAM3X8E_TWI.TWI0
  with Preelaborate, Elaborate_Body
is

   TWI0 : aliased TWI0_Controller;

end A0B.I2C.ATSAM3X8E_TWI.TWI0;