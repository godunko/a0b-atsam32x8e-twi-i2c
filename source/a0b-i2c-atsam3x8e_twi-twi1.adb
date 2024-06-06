--
--  Copyright (C) 2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  pragma Restrictions (No_Elaboration_Code);

package body A0B.I2C.ATSAM3X8E_TWI.TWI1 is

   procedure TWI1_Handler
     with Export, Convention => C, External_Name => "TWI1_Handler";

   ------------------
   -- TWI1_Handler --
   ------------------

   procedure TWI1_Handler is
   begin
      TWI1.On_Interrupt;
   end TWI1_Handler;

end A0B.I2C.ATSAM3X8E_TWI.TWI1;