--
--  Copyright (C) 2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  pragma Restrictions (No_Elaboration_Code);

package body A0B.I2C.ATSAM3X8E_TWI.TWI0 is

   procedure TWI0_Handler
     with Export, Convention => C, External_Name => "TWI0_Handler";

   ------------------
   -- TWI0_Handler --
   ------------------

   procedure TWI0_Handler is
   begin
      TWI0.On_Interrupt;
   end TWI0_Handler;

end A0B.I2C.ATSAM3X8E_TWI.TWI0;