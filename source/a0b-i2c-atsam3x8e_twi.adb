--
--  Copyright (C) 2024, Vadim Godunko <vgodunko@gmail.com>
--
--  SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
--

pragma Restrictions (No_Elaboration_Code);
pragma Ada_2022;

--  with Ada.Unchecked_Conversion;
with System.Address_To_Access_Conversions;
pragma Warnings (Off, """System.Atomic_Primitives"" is an internal GNAT unit");
with System.Atomic_Primitives;
with System.Storage_Elements;

with A0B.ARMv7M.NVIC_Utilities;
with A0B.ATSAM3X8E.SVD.PMC; use A0B.ATSAM3X8E.SVD.PMC;

package body A0B.I2C.ATSAM3X8E_TWI
  with Preelaborate
is

--     procedure Configure_Target_Address
--       (Self : in out Master_Controller'Class);
--     --  Configure target address.

--     ------------------------------
--     -- Configure_Target_Address --
--     ------------------------------

--     procedure Configure_Target_Address
--       (Self : in out Master_Controller'Class)
--     is
--        Target_Address : constant Device_Address :=
--          Device_Locks.Device (Self.Device_Lock).Target_Address;

--     begin
--        if Self.State = Unused then
--           --  Set device address and addressing mode to do its only once.

--           declare
--              Val : A0B.SVD.STM32H723.I2C.CR2_Register := Self.Peripheral.CR2;

--           begin
--              if Target_Address <= 16#7F# then
--                 Val.ADD10    := False;
--                 Val.SADD.Val :=
--                   A0B.Types.Unsigned_10
--                     (A0B.Types.Shift_Left
--                        (A0B.Types.Unsigned_32 (Target_Address), 1));
--                 --  In 7-bit addressing mode device address should be written
--                 --  to SADD[7:1], so shift it left by one bit.

--              else
--                 Val.ADD10    := True;
--                 Val.SADD.Val :=
--                   A0B.Types.Unsigned_10 (Target_Address);
--                   --    (A0B.Types.Shift_Left
--                   --       (A0B.Types.Unsigned_8 (Device.Address), 1));
--                 --  In 7-bit addressing mode device address should be written to
--                 --  SADD[7:1], so shift it left by one bit.
--              end if;

--              Self.Peripheral.CR2 := Val;
--           end;

--           Self.State := Configured;
--        end if;
--     end Configure_Target_Address;

   ---------------
   -- Configure --
   ---------------

   procedure Configure (Self : in out Master_Controller'Class) is
   begin
      --  Enable peripheral clock

      PMC_Periph.PMC_PCER0.PID.Arr (Integer (Self.Identifier)) := True;

      --  Reset peripheral controller

      Self.Peripheral.CR := (SWRST => True, others => <>);

   --        Dummy_Byte : BBF.HRI.Byte;
   --
   --  begin
   --     Self.CR.SWRST := True;
   --     --  Set SWRST bit to reset TWI peripheral
   --
   --     Dummy_Byte := Self.RHR.RXDATA;
   --  --  Disable transfers and interrupts

      --  Self.Peripheral.PTCR :=
      --    (RXTDIS => True,
      --     TXTDIS => True,
      --     others => <>);
      --  Self.Peripheral.IDR :=
      --    (TXCOMP         => True,
      --     RXRDY          => True,
      --     TXRDY          => True,
      --     SVACC          => True,
      --     GACC           => True,
      --     OVRE           => True,
      --     NACK           => True,
      --     ARBLST         => True,
      --     SCL_WS         => True,
      --     EOSACC         => True,
      --     ENDRX          => True,
      --     ENDTX          => True,
      --     RXBUFF         => True,
      --     TXBUFE         => True,
      --     others => <>);

      Self.Peripheral.CR := (MSEN => True, others => <>);
      Self.Peripheral.CWGR :=
        (CLDIV  => 109,
         CHDIV  => 96,
         CKDIV  => 0,
         others => <>);
      --  About 400_000 fast mode.
      --
      --  XXX Should be configurable.

      --  Configure PIO pins

      Self.TWCK.Configure_Alternative_Function (Self.TWCK_Function);
      Self.TWD.Configure_Alternative_Function (Self.TWD_Function);

      --  Clear pending status and enable NVIC interrupts.

      declare
         Aux : constant A0B.ATSAM3X8E.SVD.TWI.TWI0_SR_Register :=
           Self.Peripheral.SR;
         --  Dummy read to clear status.

      begin
         null;
      end;

      A0B.ARMv7M.NVIC_Utilities.Clear_Pending (Self.Interrupt);
      A0B.ARMv7M.NVIC_Utilities.Enable_Interrupt (Self.Interrupt);

      --  Initialize and configure TWI controller in master mode

   --     Dummy_SR      : BBF.HRI.TWI.TWI0_SR_Register;
   --     Dummy_Boolean : Boolean;
   --
   --  begin
   --     --  Disable TWI interrupts
   --
   --     Disable_Interrupts (Self);
   --
   --     --  Reset TWI peripheral
   --
   --     Reset (Self);
   --
   --     Enable_Master_Mode (Self);
   --
   --     --  Select the speed
   --     Set_Speed (Self, Main_Clock_Frequency, Speed, Dummy_Boolean);
   --
   --     --  XXX For SPI mode CR.QUICK must be set
      --  BBF.HPL.TWI.Initialize_Master
      --    (Self.Controller, BBF.BSL.Clocks.Main_Clock_Frequency, 384_000);

--        --  Software reset I2C
--        --
--        --  [RM0468] 52.4.6 Software reset
--        --
--        --  "PE must be kept low during at least 3 APB clock cycles in order to
--        --  perform the software reset. This is ensured by writing the following
--        --  software sequence:
--        --   - Write PE=0
--        --   - Check PE=0
--        --   - Write PE=1."
--        --
--        --  Note, I2C must be disabled to be able to configure some parameters
--        --  (ANFOF, DNF, TIMING), so write PE=1 is not needed. This register will
--        --  be read to preserve state of the reserved bits later, thus nothing to
--        --  do here.

--        Self.Peripheral.CR1.PE := False;

--        --  Configure control register 1

--        declare
--           Val : A0B.SVD.STM32H723.I2C.CR1_Register := Self.Peripheral.CR1;

--        begin
--           Val.PECEN     := False;
--           Val.ALERTEN   := False;
--           Val.SMBDEN    := False;
--           --  Device default address disabled (I2C mode)
--           Val.SMBHEN    := False;    --  Host address disabled (I2C mode)
--           Val.GCEN      := False;
--           Val.WUPEN     := False;
--           Val.NOSTRETCH := False;    --  Must be kept cleared in master mode
--           Val.SBC       := False;
--           Val.RXDMAEN   := False;    --  RX DMA disabled
--           Val.TXDMAEN   := False;    --  TX DMA disabled
--           Val.ANFOFF    := False;    --  Analog filter enabled
--           Val.DNF       := 2#0000#;  --  Digital filter disabled
--           Val.ERRIE     := True;     --  Error interrupt enabled
--           Val.TCIE      := True;     --  Transfer Complete interrupt enabled
--           Val.STOPIE    := True;
--           --  Stop detection (STOPF) interrupt enabled
--           Val.NACKIE    := True;
--           --  Not acknowledge (NACKF) received interrupts enabled
--           Val.ADDRIE    := False;
--           --  Address match (ADDR) interrupts disabled
--           Val.RXIE      := True;     --  Receive (RXNE) interrupt enabled
--           Val.TXIE      := True;     --  Transmit (TXIS) interrupt enabled

--           Self.Peripheral.CR1 := Val;
--        end;

--        --  Configure timing register (Fast Mode)

--        declare
--           Val : A0B.SVD.STM32H723.I2C.TIMINGR_Register :=
--             Self.Peripheral.TIMINGR;

--        begin
--           --  Standard Mode

--           Val.PRESC  := 16#2#;
--           Val.SCLDEL := 16#A#;
--           Val.SDADEL := 16#0#;
--           Val.SCLH   := 16#AC#;
--           Val.SCLL   := 16#FE#;

--           --  Fast Mode
--           --  Val.PRESC  := 0;
--           --  Val.SCLDEL := 16#C#;
--           --  Val.SDADEL := 0;
--           --  Val.SCLH   := 16#45#;
--           --  Val.SCLL   := 16#ED#;

--           Self.Peripheral.TIMINGR := Val;
--        end;

--        --  Enable I2C

--        Self.Peripheral.CR1.PE := True;

--        --  Clear pending and enable NVIC interrupts

--        A0B.ARMv7M.NVIC_Utilities.Clear_Pending (Self.Event_Interrupt);
--        A0B.ARMv7M.NVIC_Utilities.Clear_Pending (Self.Error_Interrupt);
--        A0B.ARMv7M.NVIC_Utilities.Enable_Interrupt (Self.Event_Interrupt);
--        A0B.ARMv7M.NVIC_Utilities.Enable_Interrupt (Self.Error_Interrupt);
   end Configure;

   ------------------
   -- Device_Locks --
   ------------------

   package body Device_Locks is

      function Atomic_Compare_Exchange is
        new System.Atomic_Primitives.Atomic_Compare_Exchange
              (System.Storage_Elements.Integer_Address);

      package Conversions is
        new System.Address_To_Access_Conversions
              (Abstract_I2C_Device_Driver'Class);

      -------------
      -- Acquire --
      -------------

      procedure Acquire
        (Self    : in out Lock;
         Device  : not null I2C_Device_Driver_Access;
         Success : in out Boolean)
      is
         Aux : System.Storage_Elements.Integer_Address :=
           System.Storage_Elements.To_Integer (System.Null_Address);

      begin
         if not Success
           or else (Self.Device /= null and Self.Device /= Device)
         then
            Success := False;

            return;
         end if;

         if Self.Device /= null then
            return;
         end if;

         if not Atomic_Compare_Exchange
           (Ptr      => Self.Device'Address,
            Expected => Aux'Address,
            Desired  =>
              System.Storage_Elements.To_Integer
                (Conversions.To_Address
                   (Conversions.Object_Pointer (Device))))
         then
            Success := False;

            return;
         end if;

      end Acquire;

      -------------
      -- Release --
      -------------

      procedure Release
        (Self    : in out Lock;
         Device  : not null I2C_Device_Driver_Access;
         Success : in out Boolean) is
      begin
         if not Success
           or else Self.Device /= Device
         then
            Success := False;

            return;
         end if;

         Self.Device := null;
      end Release;

   end Device_Locks;

   ------------------
   -- On_Interrupt --
   ------------------

   procedure On_Interrupt (Self : in out Master_Controller'Class) is

      use type A0B.Types.Unsigned_32;

--        --  function Pending_Interrupts
--        --    (Status : A0B.SVD.STM32H723.I2C.ISR_Register;
--        --     Mask   : A0B.SVD.STM32H723.I2C.CR1_Register)
--        --     return A0B.SVD.STM32H723.I2C.ISR_Register;
--        --
--        --  ------------------------
--        --  -- Pending_Interrupts --
--        --  ------------------------
--        --
--        --  function Pending_Interrupts
--        --    (Status : A0B.SVD.STM32H723.I2C.ISR_Register;
--        --     Mask   : A0B.SVD.STM32H723.I2C.CR1_Register)
--        --     return A0B.SVD.STM32H723.I2C.ISR_Register
--        --  is
--        --     function To_Unsigned_32 is
--        --       new Ada.Unchecked_Conversion
--        --             (A0B.SVD.STM32H723.I2C.ISR_Register, A0B.Types.Unsigned_32);
--        --
--        --     function To_Unsigned_32 is
--        --       new Ada.Unchecked_Conversion
--        --             (A0B.SVD.STM32H723.I2C.CR1_Register, A0B.Types.Unsigned_32);
--        --
--        --     function To_ISR_Register is
--        --       new Ada.Unchecked_Conversion
--        --         (A0B.Types.Unsigned_32, A0B.SVD.STM32H723.I2C.ISR_Register);
--        --
--        --  begin
--        --     return
--        --       To_ISR_Register
--        --         (To_Unsigned_32 (Status) and To_Unsigned_32 (Mask));
--        --  end Pending_Interrupts;

--        Status  : constant A0B.SVD.STM32H723.I2C.ISR_Register :=
--          Self.Peripheral.ISR;
--        Mask    : constant A0B.SVD.STM32H723.I2C.CR1_Register :=
--          Self.Peripheral.CR1;
--        --  Pending : constant A0B.SVD.STM32H723.I2C.ISR_Register :=
--        --    Pending_Interrupts (Status, Mask);

      Status : constant A0B.ATSAM3X8E.SVD.TWI.TWI0_SR_Register :=
        Self.Peripheral.SR;
      Mask   : constant A0B.ATSAM3X8E.SVD.twi.TWI0_IMR_Register :=
        Self.Peripheral.IMR;
      Size   : A0B.Types.Unsigned_32;

   begin
      --  raise Program_Error;

      --     Status : constant BBF.HPL.TWI.TWI_Status :=
      --       BBF.HPL.TWI.Get_Masked_Status (Driver.Controller);
      --
      --  begin
      --     ---------------------------
      --     --  Receive Buffer Full  --
      --     ---------------------------
      --
      --     if BBF.HPL.TWI.Is_Receive_Buffer_Full (Status) then
      --        --  Disable transfer and interrupt.
      --
      --        BBF.HPL.TWI.Disable_Receive_Buffer (Driver.Controller);
      --        BBF.HPL.TWI.Disable_Interrupt
      --          (Driver.Controller, BBF.HPL.TWI.Receive_Buffer_Full);
      --
      --        --  Enable Receive_Holding_Register_Ready interrupt to send STOP
      --        --  condition.
      --
      --        BBF.HPL.TWI.Enable_Interrupt
      --          (Driver.Controller, BBF.HPL.TWI.Receive_Holding_Register_Ready);
      --     end if;
      --
      --     --------------------------------------
      --     --  Receive Holding Register Ready  --
      --     --------------------------------------
      --
      --     if BBF.HPL.TWI.Is_Receive_Holding_Register_Ready (Status) then
      --        declare
      --           Data : BBF.Unsigned_8_Array_16 (1 .. Driver.Current.Length)
      --             with Address => Driver.Current.Data;
      --
      --        begin
      --           if Driver.Current.Stop then
      --              --  Store last byte
      --
      --              Data (Data'Last) :=
      --                BBF.Unsigned_8 (Driver.Controller.RHR.RXDATA);
      --
      --              --  Disable interrupt and wait till transmission completed
      --
      --              BBF.HPL.TWI.Disable_Interrupt
      --                (Driver.Controller,
      --                 BBF.HPL.TWI.Receive_Holding_Register_Ready);
      --
      --           else
      --              --  Store last but one byte
      --
      --              Data (Data'Last - 1) :=
      --                BBF.Unsigned_8 (Driver.Controller.RHR.RXDATA);
      --
      --              --  Send STOP condition.
      --
      --              BBF.HPL.TWI.Send_Stop_Condition (Driver.Controller);
      --              Driver.Current.Stop := True;
      --           end if;
      --        end;
      --     end if;

      --  Transmit Buffer Empty

      if Status.TXBUFE and Mask.TXBUFE then
         --  Transfer of the buffer has been completed successfully. Disable
         --  PDC transmissions, TXBUFE interrupt and update status of the
         --  transmitted buffer.

         Self.Peripheral.IDR := (TXBUFE => True, others => <>);
         Self.Peripheral.PTCR.TXTDIS := True;

         Self.Buffer.Transferred := Self.Buffer.Size;
         Self.Buffer.State       := Success;

         if Self.Next <= Self.Buffers'Last then
            --  Initiate transmission of the next buffer.

            Self.Buffer := Self.Buffers (Self.Next)'Access;
            Self.Next   := @ + 1;

            --  Initiate new PDC transfer

            Self.Peripheral.TPR :=
              A0B.Types.Unsigned_32
                (System.Storage_Elements.To_Integer
                   (Self.Buffer.Address));
            Self.Peripheral.TCR :=
              (TXCTR  => A0B.Types.Unsigned_16 (Self.Buffer.Size),
               others => <>);

            Self.Peripheral.IER := (TXBUFE => True, others => <>);
            Self.Peripheral.PTCR.TXTEN := True;

         else
            --  All buffers has been transmitted, notify driver and send STOP
            --  condition when requested.

            Self.Buffer := null;

            --  Notify driver about end of the transfer

            Device_Locks.Device (Self.Device_Lock).On_Transfer_Completed;

            if Self.Stop then
               Self.Peripheral.CR := (STOP => True, others => <>);
            end if;
         end if;
      end if;

      --     if BBF.HPL.TWI.Is_Transmit_Buffer_Empty (Status) then
      --        --  Transfer almost completed, enable Transmit Holding Register
      --        --  Ready interrupt to send stop condition.
      --
      --        BBF.HPL.TWI.Enable_Interrupt
      --          (Driver.Controller,
      --           BBF.HPL.TWI.Transmit_Holding_Register_Ready);
      --
      --        --  Disable transfer and interrupt.
      --
      --        BBF.HPL.TWI.Disable_Transmission_Buffer (Driver.Controller);
      --        BBF.HPL.TWI.Disable_Interrupt
      --          (Driver.Controller, BBF.HPL.TWI.Transmit_Buffer_Empty);
      --     end if;
      --
      --     --------------------------------------
      --     --  Trasmit Holding Register Ready  --
      --     --------------------------------------
      --
      --     if BBF.HPL.TWI.Is_Transmit_Holding_Register_Ready (Status) then
      --        --  Send STOP condition and last byte of the data.
      --
      --        BBF.HPL.TWI.Send_Stop_Condition (Driver.Controller);
      --
      --        --  Disable interrupt
      --
      --        BBF.HPL.TWI.Disable_Interrupt
      --          (Driver.Controller,
      --           BBF.HPL.TWI.Transmit_Holding_Register_Ready);
      --     end if;

      --  Transmission Completed

      if Status.TXCOMP and Mask.TXCOMP then
         null;
      --     if BBF.HPL.TWI.Is_Transmission_Completed (Status) then
      --        --  Disable transfers and all interrupts.
      --
      --        BBF.HPL.TWI.Disable_Receive_Buffer (Driver.Controller);
      --        BBF.HPL.TWI.Disable_Transmission_Buffer (Driver.Controller);
      --
      --        BBF.HPL.TWI.Disable_Interrupt
      --          (Driver.Controller, BBF.HPL.TWI.Receive_Buffer_Full);
      --        BBF.HPL.TWI.Disable_Interrupt
      --          (Driver.Controller, BBF.HPL.TWI.Transmit_Buffer_Empty);
      --        BBF.HPL.TWI.Disable_Interrupt
      --          (Driver.Controller, BBF.HPL.TWI.Receive_Holding_Register_Ready);
      --        BBF.HPL.TWI.Disable_Interrupt
      --          (Driver.Controller, BBF.HPL.TWI.Transmit_Holding_Register_Ready);
      --        BBF.HPL.TWI.Disable_Interrupt
      --          (Driver.Controller, BBF.HPL.TWI.Transmission_Completed);
      --        Driver.Disable_Error_Interrupts;

         if Self.Buffer = null then
            if Self.Next = Self.Buffers'First then
               --  Compute size of the transfer.

               Size := 0;

               for Buffer of Self.Buffers.all loop
                  Size := @ + Buffer.Size;
               end loop;

               --  Select first buffer.

               Self.Buffer := Self.Buffers (Self.Next)'Access;
               Self.Next   := @ + 1;

               --  Configure device address

               if Device_Locks.Device (Self.Device_Lock).Target_Address
                    <= 16#7F#
               then
                  --  7bit device address

                  Self.Peripheral.MMR :=
                    (IADRSZ => A0B.ATSAM3X8E.SVD.TWI.NONE,
                     MREAD  => False,
                     DADR   =>
                       A0B.Types.Unsigned_7
                         (Device_Locks.Device
                            (Self.Device_Lock).Target_Address),
                     others => <>);

                  --  raise Program_Error;
      --                 Driver.Controller.MMR := (others => <>);
      --                 Driver.Controller.MMR :=
      --                   (DADR   =>
      --                      BBF.HRI.TWI.TWI0_MMR_DADR_Field (Driver.Current.Device),
      --                    MREAD  => False,
      --                    IADRSZ => BBF.HRI.TWI.Val_1_Byte,
      --                    others => <>);

               --  Self.TPR :=
               --    BBF.HRI.UInt32 (System.Storage_Elements.To_Integer (Buffer));
               --  Self.TCR := (TXCTR => BBF.HRI.UInt16 (Length), others => <>);

               else
                  --  Self.Peripheral.MMR :=
                  --    (IADRSZ => NONE,
                  --     MREAD  => False,
                  --     DADR   =>
                  --       A0B.Types.Unsigned_7
                  --         (Device_Locks.Device
                  --            (Self.Device_Lock).Target_Address),
                  --     others => <>);

      --                 --  Set internal address for remote chip
      --
      --                 Driver.Controller.IADR := (others => <>);
      --                 Driver.Controller.IADR :=
      --                   (IADR   =>
      --                      BBF.HRI.TWI.TWI0_IADR_IADR_Field (Driver.Current.Register),
      --                    others => <>);
                  raise Program_Error;
               end if;

               --  Initiate data transfer

               if Self.Buffer.Size = 0 then
               --  Start transfer by setting START.

                  raise Program_Error;

               elsif Self.Buffer.Size = 1 then
                  --  Setup PDC

                  Self.Peripheral.TPR :=
                    A0B.Types.Unsigned_32
                      (System.Storage_Elements.To_Integer
                         (Self.Buffer.Address));
                  Self.Peripheral.TCR :=
                    (TXCTR  => A0B.Types.Unsigned_16 (Self.Buffer.Size),
                     others => <>);

                  --  Enable TXBUFE interrupt

                  Self.Peripheral.IER := (TXBUFE => True, others => <>);

                  --  Start ransmission by enabling PDC transfer.

                  Self.Peripheral.PTCR.TXTEN := True;

                  --  raise Program_Error;

               else
                  raise Program_Error;
               end if;
               --  if Size <= 1
               --  Self.
               --  if Self.Buffer.Size <= 1 then
               --     --  Single byte transfer is not implemented.  ??? Why ???
               --
               --     raise Program_Error;
               --  end if;

      --              if Driver.Current.Length = 1 then
      --                 raise Program_Error with "1 byte I2C write not implemented";
      --
      --              else
      --                 BBF.HPL.TWI.Set_Transmission_Buffer
      --                   (Driver.Controller,
      --                    Driver.Current.Data,
      --                    Driver.Current.Length);
      --
      --                 --  Set write mode, slave address and 3 internal address byte
      --                 --  lengths
      --
      --                 Driver.Controller.MMR := (others => <>);
      --                 Driver.Controller.MMR :=
      --                   (DADR   =>
      --                      BBF.HRI.TWI.TWI0_MMR_DADR_Field (Driver.Current.Device),
      --                    MREAD  => False,
      --                    IADRSZ => BBF.HRI.TWI.Val_1_Byte,
      --                    others => <>);
      --
      --                 --  Set internal address for remote chip
      --
      --                 Driver.Controller.IADR := (others => <>);
      --                 Driver.Controller.IADR :=
      --                   (IADR   =>
      --                      BBF.HRI.TWI.TWI0_IADR_IADR_Field (Driver.Current.Register),
      --                    others => <>);
      --
      --                 --  Enable interrupts and transfer
      --
      --                 BBF.HPL.TWI.Enable_Transmission_Buffer (Driver.Controller);
      --                 Driver.Enable_Error_Interrupts;
      --                 BBF.HPL.TWI.Enable_Interrupt
      --                   (Driver.Controller, BBF.HPL.TWI.Transmit_Buffer_Empty);
      --                 BBF.HPL.TWI.Enable_Interrupt
      --                   (Driver.Controller, BBF.HPL.TWI.Transmission_Completed);
      --              end if;
            else
               --  Transaction has been completed for some reason.

               declare
                  Device  : constant I2C_Device_Driver_Access :=
                    Device_Locks.Device (Self.Device_Lock);
                  Success : Boolean := True;

               begin
                  Device_Locks.Release (Self.Device_Lock, Device, Success);
                  pragma Assert (Success);

                  Self.Buffers := null;
                  --  Self.State := Unused;

                  --  Disable Transmission_Completed interrupt to avoid
                  --  endless interrupt calls.

                  Self.Peripheral.IDR := (TXCOMP => True, others => <>);

                  Device.On_Transaction_Completed;
               end;
            end if;

         else
            raise Program_Error;
         end if;
      --        if Driver.Current.Operation /= None then
      --           if BBF.HPL.TWI.Is_Overrun_Error (Status)
      --             or BBF.HPL.TWI.Is_Not_Acknowledge (Status)
      --             or (BBF.HPL.TWI.Is_Arbitration_Lost (Status)
      --                   and Driver.Current.Retry = 0)
      --           then
      --              if Driver.Current.On_Error /= null then
      --                 Driver.Current.On_Error (Driver.Current.Closure);
      --              end if;
      --
      --              Driver.Current := (Operation => None);
      --
      --           elsif BBF.HPL.TWI.Is_Not_Acknowledge (Status) then
      --              --  Arbitration lost, try to retry operation for few times.
      --
      --              Driver.Current.Retry := @ - 1;
      --              Driver.Current.Stop  := False;
      --
      --           else
      --              if Driver.Current.On_Success /= null then
      --                 Driver.Current.On_Success (Driver.Current.Closure);
      --              end if;
      --
      --              Driver.Current := (Operation => None);
      --           end if;
      --        end if;
      --
      --        --  When there is no operation in progress, attempt to dequeue
      --        --  next one.
      --
      --        if Driver.Current.Operation = None then
      --           if not Operation_Queues.Dequeue (Driver.Queue, Driver.Current) then
      --              return;
      --           end if;
      --        end if;
      --
      --        --  Initiate operation.
      --
      --        case Driver.Current.Operation is
      --           when Read =>
      --              if Driver.Current.Length >= 2 then
      --                 --  Last two bytes are send by the interrupt handler on
      --                 --  Receive Holding Register Ready interrupt.
      --
      --                 BBF.HPL.TWI.Set_Receive_Buffer
      --                   (Driver.Controller,
      --                    Driver.Current.Data,
      --                    Driver.Current.Length - 2);
      --              end if;
      --
      --              --  Set read mode, slave address and 3 internal address byte
      --              --  lengths
      --
      --              Driver.Controller.MMR := (others => <>);
      --              Driver.Controller.MMR :=
      --                (DADR   =>
      --                   BBF.HRI.TWI.TWI0_MMR_DADR_Field (Driver.Current.Device),
      --                 MREAD  => True,
      --                 IADRSZ => BBF.HRI.TWI.Val_1_Byte,
      --                 others => <>);
      --
      --              --  Set internal address for remote chip
      --
      --              Driver.Controller.IADR := (others => <>);
      --              Driver.Controller.IADR :=
      --                (IADR   =>
      --                   BBF.HRI.TWI.TWI0_IADR_IADR_Field (Driver.Current.Register),
      --                 others => <>);
      --
      --              --  Enable transfer and interrupts
      --
      --              if Driver.Current.Length >= 2 then
      --                 BBF.HPL.TWI.Enable_Receive_Buffer (Driver.Controller);
      --                 BBF.HPL.TWI.Enable_Interrupt
      --                   (Driver.Controller, BBF.HPL.TWI.Receive_Buffer_Full);
      --
      --              else
      --                 BBF.HPL.TWI.Enable_Interrupt
      --                   (Driver.Controller,
      --                    BBF.HPL.TWI.Receive_Holding_Register_Ready);
      --              end if;
      --
      --              Driver.Enable_Error_Interrupts;
      --              BBF.HPL.TWI.Enable_Interrupt
      --                (Driver.Controller, BBF.HPL.TWI.Transmission_Completed);
      --
      --              --  Send a START condition
      --
      --              if Driver.Current.Length = 1 then
      --                 --  Send both START and STOP conditions.
      --
      --                 Driver.Controller.CR :=
      --                   (START => True, STOP => True, others => <>);
      --                 Driver.Current.Stop  := True;
      --
      --              else
      --                 --  Otherwise, send only START condition.
      --
      --                 Driver.Controller.CR := (START => True, others => <>);
      --              end if;
      --
      --           when Write =>
      --              if Driver.Current.Length = 1 then
      --                 raise Program_Error with "1 byte I2C write not implemented";
      --
      --              else
      --                 BBF.HPL.TWI.Set_Transmission_Buffer
      --                   (Driver.Controller,
      --                    Driver.Current.Data,
      --                    Driver.Current.Length);
      --
      --                 --  Set write mode, slave address and 3 internal address byte
      --                 --  lengths
      --
      --                 Driver.Controller.MMR := (others => <>);
      --                 Driver.Controller.MMR :=
      --                   (DADR   =>
      --                      BBF.HRI.TWI.TWI0_MMR_DADR_Field (Driver.Current.Device),
      --                    MREAD  => False,
      --                    IADRSZ => BBF.HRI.TWI.Val_1_Byte,
      --                    others => <>);
      --
      --                 --  Set internal address for remote chip
      --
      --                 Driver.Controller.IADR := (others => <>);
      --                 Driver.Controller.IADR :=
      --                   (IADR   =>
      --                      BBF.HRI.TWI.TWI0_IADR_IADR_Field (Driver.Current.Register),
      --                    others => <>);
      --
      --                 --  Enable interrupts and transfer
      --
      --                 BBF.HPL.TWI.Enable_Transmission_Buffer (Driver.Controller);
      --                 Driver.Enable_Error_Interrupts;
      --                 BBF.HPL.TWI.Enable_Interrupt
      --                   (Driver.Controller, BBF.HPL.TWI.Transmit_Buffer_Empty);
      --                 BBF.HPL.TWI.Enable_Interrupt
      --                   (Driver.Controller, BBF.HPL.TWI.Transmission_Completed);
      --              end if;
      --
      --           when None =>
      --              raise Program_Error;
      --        end case;
      end if;


--        --  if Status.TXIS and Mask.TXIE then
--        --  if Status.TXIS and not Self.Peripheral.CR2.RD_WRN then
--        if Status.TXIS then
--           --  raise Program_Error;
--           Self.Peripheral.TXDR.TXDATA := Self.Buffer (Self.Status.Bytes);
--           Self.Status.Bytes           := @ + 1;
--        end if;

--        if Status.RXNE then
--           Self.Buffer (Self.Status.Bytes) := Self.Peripheral.RXDR.RXDATA;
--           Self.Status.Bytes               := @ + 1;
--        end if;

--        if Status.TC and Mask.TCIE then
--           Self.Peripheral.CR1.TCIE := False;
--           --  Disable TCR and TC interrupts, software should write to NBYTES
--           --  to clear this flag. It will be re-enabled after this write by
--           --  Write/Read procedure.

--           --  --  Disable TC interrupt, it is active till master sends START/STOP
--           --  --  condition. Device driver need to be notified only once, and
--           --  --  there is nothing to do till ball is on device driver's side.
--           --  --
--           --  --  This supports case when driver can't release controller or
--           --  --  initiate next transfer immidiately.

--           --  raise Program_Error;
--           Self.Status.State := Success;
--           Device_Locks.Device (Self.Device_Lock).On_Transfer_Completed;
--        end if;

--        if Status.NACKF then
--           --  raise Program_Error;
--           Self.Peripheral.ICR.NACKCF := True;

--           if not Status.TXE then
--              --  Byte was not transmitted, decrement counter and flush transmit
--              --  data regitser.

--              Self.Status.Bytes := @ - 1;
--              Self.Peripheral.ISR.TXE := True;
--           end if;

--           --  Self.Status.State := Success;  --  ???
--           Self.Status.State := Failure;  --  ???
--           Device_Locks.Device (Self.Device_Lock).On_Transfer_Completed;
--     --        Self.Release_Device;
--     --        --  Self.Busy := False;
--     --        --  raise Program_Error;
--        end if;
--     --
--        if Status.STOPF then
--           Self.Peripheral.ICR.STOPCF := True;
--           --  Clear STOPF interrupt status

--           declare
--              Device  : constant I2C_Device_Driver_Access :=
--                Device_Locks.Device (Self.Device_Lock);
--              Success : Boolean := True;

--           begin
--              Device_Locks.Release (Self.Device_Lock, Device, Success);
--              Self.State := Unused;

--              Device.On_Transaction_Completed;
--           end;
--        end if;

--        -----------------------------------------------------------------------

--        if Status.TCR and Mask.TCIE then
--           raise Program_Error;
--           --  Self.Peripheral.CR1.TCIE := False;
--           --  --  Disable TCR and TC interrupts, software should write to NBYTES
--           --  --  to clear this flag. It will be re-enabled after this write by
--           --  --  Write/Read procedure.
--           --
--           --  Self.Status.State := Success;
--           --  Device_Locks.Device (Self.Device_Lock).On_Transfer_Completed;
--        end if;

--     --     --  if Self.Peripheral.ISR.ADDR then
--     --     --     raise Program_Error;
--     --     --  end if;
--     --
--  --  begin
--        --  null;
--        --  raise Program_Error;
   end On_Interrupt;

   ----------
   -- Read --
   ----------

   overriding procedure Read
     (Self    : in out Master_Controller;
      Device  : not null I2C_Device_Driver_Access;
      Buffers : in out Buffer_Descriptor_Array;
      Stop    : Boolean;
      Success : in out Boolean) is
   begin
      raise Program_Error;
--        Device_Locks.Acquire (Self.Device_Lock, Device, Success);

--        Self.Configure_Target_Address;

--        Self.Buffer :=
--          (if Buffer'Length = 0 then null else Buffer'Unrestricted_Access);
--        Self.Status := Status'Unchecked_Access;
--        Self.State  := Read;

--        Self.Status.all := (Bytes => 0, State => Active);

--        A0B.ARMv7M.NVIC_Utilities.Disable_Interrupt (Self.Event_Interrupt);
--        --  Disable event interrup from the peripheral controller to prevent
--        --  undesired TC interrupt (it will be cleared by send of the START
--        --  condition).

--        Self.Peripheral.CR1.TCIE := True;
--        --  Enable TC and TCE interrupts.

--        --  Set transfer parameters and send (Re)START condition.

--        declare
--           Val : A0B.SVD.STM32H723.I2C.CR2_Register := Self.Peripheral.CR2;

--        begin
--           Val.RD_WRN  := True;           --  Master requests a read transfer.
--           Val.NBYTES  := Buffer'Length;  --  Number of bytes to be transfered.

--           Val.AUTOEND := False;
--           Val.RELOAD  := False;
--           Val.START   := True;
--           --  Val.RELOAD  := True;
--           --  Val.START   := Self.State /= Read;
--           --  if Self.State /= Read then

--           --
--           --     --  Send (Re)START condition
--           --
--           --     Val.START := True;
--           --  end if;

--           Self.Peripheral.CR2 := Val;
--        end;

--        A0B.ARMv7M.NVIC_Utilities.Clear_Pending (Self.Event_Interrupt);
--        A0B.ARMv7M.NVIC_Utilities.Enable_Interrupt (Self.Event_Interrupt);
--        --  Clear pending interrupt status and enable interrupt.

--        --  Self.Peripheral.CR1.TCIE := True;

--        --  Self.Transfer :=
--        --    (Operation => Read,
--        --     Buffer    => Self.Read_Buffer,
--        --     Index     => 0);
--        --
--        --  --  Self.Controller.Peripheral.CR1.TCIE := True;
--        --
--        --  --  Prepare to read and send ReSTART condition
--        --
--        --  declare
--        --     Val : A0B.SVD.STM32H723.I2C.CR2_Register :=
--        --       Self.Controller.Peripheral.CR2;
--        --
--        --  begin
--        --     Val.RD_WRN  := True;  --  Master requests a read transfer.
--        --     Val.NBYTES  := Self.Transfer.Buffer'Length;
--        --
--        --     Val.AUTOEND := False;
--        --     Val.RELOAD  := False;
--        --
--        --     Val.START   := True;
--        --
--        --     Self.Controller.Peripheral.CR2 := Val;
--        --  end;
--        --
--        --  Self.Controller.Peripheral.CR1.TCIE := True;
   end Read;

   ----------
   -- Stop --
   ----------

   overriding procedure Stop
     (Self    : in out Master_Controller;
      Device  : not null I2C_Device_Driver_Access;
      Success : in out Boolean) is
   begin
      raise Program_Error;

--        A0B.ARMv7M.NVIC_Utilities.Disable_Interrupt (Self.Event_Interrupt);
--        --  Disable event interrup from the peripheral controller to prevent
--        --  undesired TC interrupt (it will be cleared by send of the START
--        --  condition).

--        Self.Peripheral.CR1.TCIE := True;
--        --  Enable TC and TCE interrupts.

--        --  Send STOP condition.

--        Self.Peripheral.CR2.STOP := True;

--        --  declare
--        --     Val : A0B.SVD.STM32H723.I2C.CR2_Register := Self.Peripheral.CR2;
--        --
--        --  begin
--        --     Val.RD_WRN  := True;           --  Master requests a read transfer.
--        --     Val.NBYTES  := Buffer'Length;  --  Number of bytes to be transfered.
--        --
--        --     Val.AUTOEND := False;
--        --     Val.RELOAD  := False;
--        --     Val.START   := True;
--        --     --  Val.RELOAD  := True;
--        --     --  Val.START   := Self.State /= Read;
--        --     --  if Self.State /= Read then
--        --
--        --     --
--        --     --     --  Send (Re)START condition
--        --     --
--        --     --     Val.START := True;
--        --     --  end if;
--        --
--        --     Self.Peripheral.CR2 := Val;
--        --  end;

--        A0B.ARMv7M.NVIC_Utilities.Clear_Pending (Self.Event_Interrupt);
--        A0B.ARMv7M.NVIC_Utilities.Enable_Interrupt (Self.Event_Interrupt);
--        --  Clear pending interrupt status and enable interrupt.

--        --  null;
--        --  raise Program_Error;
   end Stop;

   -----------
   -- Start --
   -----------

   overriding procedure Start
     (Self    : in out Master_Controller;
      Device  : not null I2C_Device_Driver_Access;
      Success : in out Boolean) is
   begin
      Device_Locks.Acquire (Self.Device_Lock, Device, Success);

      if not Success then
         return;
      end if;

--        Self.Configure_Target_Address;
   end Start;

--     -----------
--     -- Write --
--     -----------
--
--     overriding procedure Write
--       (Self    : in out Master_Controller;
--        Device  : not null I2C_Device_Driver_Access;
--        Buffer  : aliased in out Buffer_Descriptor;
--        Stop    : Boolean;
--        Success : in out Boolean)
--     is
--  --        use type A0B.Types.Unsigned_32;
--
--     begin
--        raise Program_Error;
--        Device_Locks.Acquire (Self.Device_Lock, Device, Success);

--        Self.Configure_Target_Address;

--        Self.Buffer :=
--          (if Buffer'Length = 0 then null else Buffer'Unrestricted_Access);
--        Self.Status := Status'Unchecked_Access;

--        Self.Status.all := (Bytes => 0, State => Active);
--        Self.State := Write;

--        --  Self.Write_Buffer :=
--        --    (if Write_Buffer'Length = 0
--        --     then null
--        --     else Write_Buffer'Unrestricted_Access);
--        --  Self.Read_Buffer  := null;
--        --  Self.Done         := Done;
--        --  Self.Busy         := True;
--        --
--        --  Self.Transfer :=
--        --    (Operation => Write,
--        --     Buffer    => Self.Write_Buffer,
--        --     Index     => 0);

--        A0B.ARMv7M.NVIC_Utilities.Disable_Interrupt (Self.Event_Interrupt);
--        --  Disable event interrup from the peripheral controller to prevent
--        --  undesired TC interrupt (it will be cleared by send of the START
--        --  condition).

--        Self.Peripheral.CR1.TCIE := True;
--        --  Enable TC and TCE interrupts.

--        --  Apply workaround.
--        --
--        --  [ES0491] 2.16.4 Transmission stalled after first byte transfer
--        --
--        --  "Write the first data in I2C_TXDR before the transmission
--        --  starts."

--        if Self.Buffer /= null then
--           Self.Peripheral.TXDR.TXDATA := Self.Buffer (Self.Status.Bytes);
--           Self.Status.Bytes := @ + 1;
--        end if;

--        --  Set transfer parameters and send (Re)START condition.

--        declare
--           Val : A0B.SVD.STM32H723.I2C.CR2_Register := Self.Peripheral.CR2;

--        begin
--           Val.RD_WRN  := False;          --  Master requests a write transfer.
--           Val.NBYTES  := Buffer'Length;  --  Number of bytes to be transfered.

--           Val.AUTOEND := False;
--           Val.RELOAD  := False;
--           Val.START   := True;

--           Self.Peripheral.CR2 := Val;
--        end;

--        --  if Self.State /= Write then
--        --  end if;

--        --  Self.Peripheral.CR2.START := True;
--        --  --  Send (Re)START condition

--        A0B.ARMv7M.NVIC_Utilities.Clear_Pending (Self.Event_Interrupt);
--        A0B.ARMv7M.NVIC_Utilities.Enable_Interrupt (Self.Event_Interrupt);
--        --  Clear pending interrupt status and enable interrupt.

--        --  Self.Peripheral.CR1.TCIE := True;
--        --  --  Wait till done
--        --
--        --  while Self.Busy loop
--        --     null;
--        --  end loop;
--        --  --  while Self.Controller.Peripheral.ISR.BUSY loop
--        --  --     null;
--        --  --  end loop;
--        null;
--        --  raise Program_Error;
   --  end Write;

   -----------
   -- Write --
   -----------

   overriding procedure Write
     (Self    : in out Master_Controller;
      Device  : not null I2C_Device_Driver_Access;
      Buffers : in out Buffer_Descriptor_Array;
      Stop    : Boolean;
      Success : in out Boolean) is
   begin
      Device_Locks.Acquire (Self.Device_Lock, Device, Success);

      if not Success then
         return;
      end if;

      Self.Buffers := Buffers'Unrestricted_Access;

      for Descriptor of Self.Buffers.all loop
         Descriptor.Transferred := 0;
         Descriptor.State       := Active;
      end loop;

      Self.Next := Self.Buffers'First;
      --  Self.Buffer := null;
      Self.Stop := Stop;

      --  Enable Transmission_Completed interrupt to start operation.

      Self.Peripheral.IER := (TXCOMP => True, others => <>);

--        Self.Configure_Target_Address;

--        Self.Buffer :=
--          (if Buffer'Length = 0 then null else Buffer'Unrestricted_Access);
--        Self.Status := Status'Unchecked_Access;

--        Self.Status.all := (Bytes => 0, State => Active);
--        Self.State := Write;

--        --  Self.Write_Buffer :=
--        --    (if Write_Buffer'Length = 0
--        --     then null
--        --     else Write_Buffer'Unrestricted_Access);
--        --  Self.Read_Buffer  := null;
--        --  Self.Done         := Done;
--        --  Self.Busy         := True;
--        --
--        --  Self.Transfer :=
--        --    (Operation => Write,
--        --     Buffer    => Self.Write_Buffer,
--        --     Index     => 0);

--        A0B.ARMv7M.NVIC_Utilities.Disable_Interrupt (Self.Event_Interrupt);
--        --  Disable event interrup from the peripheral controller to prevent
--        --  undesired TC interrupt (it will be cleared by send of the START
--        --  condition).

--        Self.Peripheral.CR1.TCIE := True;
--        --  Enable TC and TCE interrupts.

--        --  Apply workaround.
--        --
--        --  [ES0491] 2.16.4 Transmission stalled after first byte transfer
--        --
--        --  "Write the first data in I2C_TXDR before the transmission
--        --  starts."

--        if Self.Buffer /= null then
--           Self.Peripheral.TXDR.TXDATA := Self.Buffer (Self.Status.Bytes);
--           Self.Status.Bytes := @ + 1;
--        end if;

--        --  Set transfer parameters and send (Re)START condition.

--        declare
--           Val : A0B.SVD.STM32H723.I2C.CR2_Register := Self.Peripheral.CR2;

--        begin
--           Val.RD_WRN  := False;          --  Master requests a write transfer.
--           Val.NBYTES  := Buffer'Length;  --  Number of bytes to be transfered.

--           Val.AUTOEND := False;
--           Val.RELOAD  := False;
--           Val.START   := True;

--           Self.Peripheral.CR2 := Val;
--        end;

--        --  if Self.State /= Write then
--        --  end if;

--        --  Self.Peripheral.CR2.START := True;
--        --  --  Send (Re)START condition

--        A0B.ARMv7M.NVIC_Utilities.Clear_Pending (Self.Event_Interrupt);
--        A0B.ARMv7M.NVIC_Utilities.Enable_Interrupt (Self.Event_Interrupt);
--        --  Clear pending interrupt status and enable interrupt.

--        --  Self.Peripheral.CR1.TCIE := True;
--        --  --  Wait till done
--        --
--        --  while Self.Busy loop
--        --     null;
--        --  end loop;
--        --  --  while Self.Controller.Peripheral.ISR.BUSY loop
--        --  --     null;
--        --  --  end loop;
--        null;
--        --  raise Program_Error;
   end Write;

end A0B.I2C.ATSAM3X8E_TWI;
