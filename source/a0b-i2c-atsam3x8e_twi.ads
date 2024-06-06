--
--  Copyright (C) 2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

--  Implementation of the I2C bus master for the ATSAM3X8E TWI controller.
--  It use PDC feature of the TWI controller for data transfer.

pragma Restrictions (No_Elaboration_Code);

with A0B.ARMv7M;
with A0B.ATSAM3X8E.SVD.TWI;
with A0B.ATSAM3X8E.PIO.PIOA;
with A0B.ATSAM3X8E.PIO.PIOB;

package A0B.I2C.ATSAM3X8E_TWI
  with Preelaborate
is

   type Master_Controller
     (Peripheral    : not null access A0B.ATSAM3X8E.SVD.TWI.TWI_Peripheral;
      Identifier    : A0B.ATSAM3X8E.Peripheral_Identifier;
      Interrupt     : A0B.ARMv7M.External_Interrupt_Number;
      TWCK          : not null access A0B.ATSAM3X8E.PIO.ATSAM3X8E_Pin'Class;
      TWCK_Function : A0B.ATSAM3X8E.Line_Function;
      TWD           : not null access A0B.ATSAM3X8E.PIO.ATSAM3X8E_Pin'Class;
      TWD_Function  : A0B.ATSAM3X8E.Line_Function) is
        limited new A0B.I2C.I2C_Bus_Master with private
          with Preelaborable_Initialization;

   procedure Configure (Self : in out Master_Controller'Class);

   subtype TWI0_Controller is Master_Controller
     (Peripheral    => A0B.ATSAM3X8E.SVD.TWI.TWI0_Periph'Access,
      Identifier    => A0B.ATSAM3X8E.Two_Wire_Interface_0,
      Interrupt     => A0B.ATSAM3X8E.Two_Wire_Interface_0,
      TWCK          => A0B.ATSAM3X8E.PIO.PIOA.PA18'Access,
      TWCK_Function => A0B.ATSAM3X8E.TWCK0,
      TWD           => A0B.ATSAM3X8E.PIO.PIOA.PA17'Access,
      TWD_Function  => A0B.ATSAM3X8E.TWD0);

   subtype TWI1_Controller is Master_Controller
     (Peripheral => A0B.ATSAM3X8E.SVD.TWI.TWI1_Periph'Access,
      Identifier => A0B.ATSAM3X8E.Two_Wire_Interface_1,
      Interrupt  => A0B.ATSAM3X8E.Two_Wire_Interface_1,
      TWCK          => A0B.ATSAM3X8E.PIO.PIOB.PB13'Access,
      TWCK_Function => A0B.ATSAM3X8E.TWCK1,
      TWD           => A0B.ATSAM3X8E.PIO.PIOB.PB12'Access,
      TWD_Function  => A0B.ATSAM3X8E.TWD1);

private

   package Device_Locks is

      type Lock is limited private with Preelaborable_Initialization;

      procedure Acquire
        (Self    : in out Lock;
         Device  : not null I2C_Device_Driver_Access;
         Success : in out Boolean);

      procedure Release
        (Self    : in out Lock;
         Device  : not null I2C_Device_Driver_Access;
         Success : in out Boolean);

      function Device (Self : Lock) return I2C_Device_Driver_Access;

   private

      type Lock is limited record
         Device : I2C_Device_Driver_Access;
      end record;

      function Device (Self : Lock) return I2C_Device_Driver_Access is
        (Self.Device);

   end Device_Locks;

   type Controller_State is
     (Unused, Configured, Read, Write, Stop_Requested);

   type Master_Controller
     (Peripheral    : not null access A0B.ATSAM3X8E.SVD.TWI.TWI_Peripheral;
      Identifier    : A0B.ATSAM3X8E.Peripheral_Identifier;
      Interrupt     : A0B.ARMv7M.External_Interrupt_Number;
      TWCK          : not null access A0B.ATSAM3X8E.PIO.ATSAM3X8E_Pin'Class;
      TWCK_Function : A0B.ATSAM3X8E.Line_Function;
      TWD           : not null access A0B.ATSAM3X8E.PIO.ATSAM3X8E_Pin'Class;
      TWD_Function  : A0B.ATSAM3X8E.Line_Function) is
   limited new I2C_Bus_Master with record
      Device_Lock : Device_Locks.Lock;
      Buffer      : access Buffer_Descriptor;
      Buffers     : access Buffer_Descriptor_Array;
      Next        : A0B.Types.Unsigned_32;
      Stop        : Boolean;
   end record;

   overriding procedure Start
     (Self    : in out Master_Controller;
      Device  : not null I2C_Device_Driver_Access;
      Success : in out Boolean);

   overriding procedure Write
     (Self    : in out Master_Controller;
      Device  : not null I2C_Device_Driver_Access;
      Buffers : in out Buffer_Descriptor_Array;
      Stop    : Boolean;
      Success : in out Boolean);

   overriding procedure Read
     (Self    : in out Master_Controller;
      Device  : not null I2C_Device_Driver_Access;
      Buffers : in out Buffer_Descriptor_Array;
      Stop    : Boolean;
      Success : in out Boolean);

   overriding procedure Stop
     (Self    : in out Master_Controller;
      Device  : not null I2C_Device_Driver_Access;
      Success : in out Boolean);

   procedure On_Interrupt (Self : in out Master_Controller'Class);

end A0B.I2C.ATSAM3X8E_TWI;
