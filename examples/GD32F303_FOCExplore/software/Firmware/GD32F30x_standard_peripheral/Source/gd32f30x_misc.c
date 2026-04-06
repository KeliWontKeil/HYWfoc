/*!
    \file    gd32f30x_misc.c
    \brief   MISC driver

    \version 2026-2-6, V3.0.3, firmware for GD32F30x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x_misc.h"

/*!
    \brief      set the priority group
    \param[in]  nvic_prigroup: the NVIC priority group
      \arg        NVIC_PRIGROUP_PRE0_SUB4:0 bits for pre-emption priority 4 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE1_SUB3:1 bits for pre-emption priority 3 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE2_SUB2:2 bits for pre-emption priority 2 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE3_SUB1:3 bits for pre-emption priority 1 bits for subpriority
      \arg        NVIC_PRIGROUP_PRE4_SUB0:4 bits for pre-emption priority 0 bits for subpriority
    \param[out] none
    \retval     none
*/
void nvic_priority_group_set(uint32_t nvic_prigroup)
{
    /* set the priority group value */
    SCB->AIRCR = NVIC_AIRCR_VECTKEY_MASK | nvic_prigroup;
}

/*!
    \brief      enable NVIC request
    \param[in]  nvic_irq: the NVIC interrupt request, detailed in IRQn_Type
    \param[in]  nvic_irq_pre_priority: the pre-emption priority needed to set
    \param[in]  nvic_irq_sub_priority: the subpriority needed to set
    \param[out] none
    \retval     none
*/
void nvic_irq_enable(IRQn_Type nvic_irq, uint8_t nvic_irq_pre_priority, 
                     uint8_t nvic_irq_sub_priority)
{
    uint32_t nvic_prigroup, nvic_priority;

    /* check current priority group */
    switch(SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) {
    case NVIC_PRIGROUP_PRE0_SUB4:
    case NVIC_PRIGROUP_PRE1_SUB3:
    case NVIC_PRIGROUP_PRE2_SUB2:
    case NVIC_PRIGROUP_PRE3_SUB1:
    case NVIC_PRIGROUP_PRE4_SUB0:
        break;
    default:
        nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
        break;
    }

    /* get the priority group value */
    nvic_prigroup = NVIC_GetPriorityGrouping();

    /* encoding the pre-emption, subpriority priority */
    nvic_priority = NVIC_EncodePriority(nvic_prigroup, (uint32_t)nvic_irq_pre_priority, (uint32_t)nvic_irq_sub_priority);
    /* set priority */
    NVIC_SetPriority(nvic_irq, nvic_priority);

    /* enable the selected IRQ */
    NVIC_EnableIRQ(nvic_irq);
}

/*!
    \brief      disable NVIC request
    \param[in]  nvic_irq: the NVIC interrupt request, detailed in IRQn_Type
    \param[out] none
    \retval     none
*/
void nvic_irq_disable(IRQn_Type nvic_irq)
{
    /* disable the selected IRQ.*/
    NVIC_DisableIRQ(nvic_irq);
}

/*!
    \brief      set the NVIC vector table base address
    \param[in]  nvic_vect_tab: the RAM or FLASH base address
      \arg        NVIC_VECTTAB_RAM: RAM base address
      \arg        NVIC_VECTTAB_FLASH: Flash base address
    \param[in]  offset: Vector Table offset
    \param[out] none
    \retval     none
*/
void nvic_vector_table_set(uint32_t nvic_vect_tab, uint32_t offset)
{
    SCB->VTOR = nvic_vect_tab | (offset & NVIC_VECTTAB_OFFSET_MASK);
    __DSB();
}

/*!
    \brief      set the state of the low power mode
    \param[in]  lowpower_mode: the low power mode state
      \arg        SCB_LPM_SLEEP_EXIT_ISR: if choose this para, the system always enter low power
                    mode by exiting from ISR
      \arg        SCB_LPM_DEEPSLEEP: if choose this para, the system will enter the DEEPSLEEP mode
      \arg        SCB_LPM_WAKE_BY_ALL_INT: if choose this para, the lowpower mode can be woken up
                    by all the enable and disable interrupts
    \param[out] none
    \retval     none
*/
void system_lowpower_set(uint8_t lowpower_mode)
{
    SCB->SCR |= (uint32_t)lowpower_mode;
}

/*!
    \brief      reset the state of the low power mode
    \param[in]  lowpower_mode: the low power mode state
      \arg        SCB_LPM_SLEEP_EXIT_ISR: if choose this para, the system will exit low power
                    mode by exiting from ISR
      \arg        SCB_LPM_DEEPSLEEP: if choose this para, the system will enter the SLEEP mode
      \arg        SCB_LPM_WAKE_BY_ALL_INT: if choose this para, the lowpower mode only can be
                    woken up by the enable interrupts
    \param[out] none
    \retval     none
*/
void system_lowpower_reset(uint8_t lowpower_mode)
{
    SCB->SCR &= (~(uint32_t)lowpower_mode);
}

/*!
    \brief      set the systick clock source
    \param[in]  systick_clksource: the systick clock source needed to choose
      \arg        SYSTICK_CLKSOURCE_HCLK: systick clock source is from HCLK
      \arg        SYSTICK_CLKSOURCE_HCLK_DIV8: systick clock source is from HCLK/8
    \param[out] none
    \retval     none
*/

void systick_clksource_set(uint32_t systick_clksource)
{
    if(SYSTICK_CLKSOURCE_HCLK == systick_clksource ){
        /* set the systick clock source from HCLK */
        SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
    }else{
        /* set the systick clock source from HCLK/8 */
        SysTick->CTRL &= SYSTICK_CLKSOURCE_HCLK_DIV8;
    }
}

#if (__MPU_PRESENT == 1)

/*!    
    \brief      enable arm mpu
    \param[in]  mpu_control: MPU control value
      \arg        MPU_MODE_HFNMI_PRIVDEF_NONE: HFNMIENA and PRIVDEFENA are 0
      \arg        MPU_MODE_HARDFAULT_NMI: use the MPU for memory accesses by HardFault and NMI handlers only
      \arg        MPU_MODE_PRIV_DEFAULT: enables the default memory map as a background region for privileged access only
      \arg        MPU_MODE_HFNMI_PRIVDEF: HFNMIENA and PRIVDEFENA are 1
    \param[out] none
    \retval     none
*/
void armv7_mpu_enable(uint32_t mpu_control)
{
  __DMB();
  MPU->CTRL = mpu_control | MPU_CTRL_ENABLE_Msk;
#ifdef SCB_SHCSR_MEMFAULTENA_Msk
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
#endif
  __DSB();
  __ISB();
}

/*!
    \brief      disable arm mpu
    \param[in]  none
    \param[out] none
    \retval     none
*/
void armv7_mpu_disable(void)
{
  __DMB();
#ifdef SCB_SHCSR_MEMFAULTENA_Msk
  SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;
#endif
  MPU->CTRL  &= ~MPU_CTRL_ENABLE_Msk;
  __DSB();
  __ISB();
}

/*!
    \brief      mpu set region
    \param[in]  rbar: rbar register value
    \param[in]  rasr: rasr register value
    \param[out] none
    \retval     none
*/
void armv7_mpu_set_region(uint32_t rbar, uint32_t rasr)
{
  MPU->RBAR = rbar;
  MPU->RASR = rasr;
}

/*!
    \brief      initialize mpu_region_init_struct with the default values
    \param[in]  mpu_init_struct: pointer to a mpu_region_init_struct structure
    \param[out] none
    \retval     none
*/
void mpu_region_struct_para_init(mpu_region_init_struct *mpu_init_struct)
{
    mpu_init_struct->region_number       = MPU_REGION_NUMBER0;
    mpu_init_struct->region_base_address = 0x00000000U;
    mpu_init_struct->instruction_exec    = MPU_INSTRUCTION_EXEC_PERMIT;
    mpu_init_struct->access_permission   = MPU_AP_NO_ACCESS;
    mpu_init_struct->tex_type            = MPU_TEX_TYPE0;
    mpu_init_struct->access_shareable    = MPU_ACCESS_SHAREABLE;
    mpu_init_struct->access_cacheable    = MPU_ACCESS_CACHEABLE;
    mpu_init_struct->access_bufferable   = MPU_ACCESS_BUFFERABLE;
    mpu_init_struct->subregion_disable   = MPU_SUBREGION_ENABLE;
    mpu_init_struct->region_size         = MPU_REGION_SIZE_32B;
}

/*!
    \brief      configure the MPU region
    \param[in]  mpu_init_struct: MPU initialization structure
                  region_number: region number
                               MPU_REGION_NUMBERn (n=0,..,7)
                  region_base_address: region base address
                  region_size: MPU_REGION_SIZE_32B, MPU_REGION_SIZE_64B, MPU_REGION_SIZE_128B, MPU_REGION_SIZE_256B, MPU_REGION_SIZE_512B, 
                               MPU_REGION_SIZE_1KB, MPU_REGION_SIZE_2KB, MPU_REGION_SIZE_4KB, MPU_REGION_SIZE_8KB, MPU_REGION_SIZE_16KB, 
                               MPU_REGION_SIZE_32KB, MPU_REGION_SIZE_64KB, MPU_REGION_SIZE_128KB, MPU_REGION_SIZE_256KB, MPU_REGION_SIZE_512KB,
                               MPU_REGION_SIZE_1MB, MPU_REGION_SIZE_2MB, MPU_REGION_SIZE_4MB, MPU_REGION_SIZE_8MB, MPU_REGION_SIZE_16MB, 
                               MPU_REGION_SIZE_32MB, MPU_REGION_SIZE_64MB, MPU_REGION_SIZE_128MB, MPU_REGION_SIZE_256MB, MPU_REGION_SIZE_512MB, 
                               MPU_REGION_SIZE_1GB, MPU_REGION_SIZE_2GB, MPU_REGION_SIZE_4GB
                  subregion_disable: MPU_SUBREGION_ENABLE, MPU_SUBREGION_DISABLE or 0x00~0xFF
                  tex_type: MPU_TEX_TYPE0, MPU_TEX_TYPE1, MPU_TEX_TYPE2 or 0x00~0x07
                  access_permission: MPU_AP_NO_ACCESS, MPU_AP_PRIV_RW, MPU_AP_PRIV_RW_UNPRIV_RO, MPU_AP_FULL_ACCESS, MPU_AP_PRIV_RO,
                                     MPU_AP_PRIV_UNPRIV_RO
                  access_shareable: MPU_ACCESS_SHAREABLE, MPU_ACCESS_NON_SHAREABLE
                  access_cacheable: MPU_ACCESS_CACHEABLE, MPU_ACCESS_NON_CACHEABLE
                  access_bufferable: MPU_ACCESS_BUFFERABLE, MPU_ACCESS_NON_BUFFERABLE
                  instruction_exec: MPU_INSTRUCTION_EXEC_PERMIT, MPU_INSTRUCTION_EXEC_NOT_PERMIT
    \param[out] none
    \retval     none
*/
void mpu_region_config(mpu_region_init_struct *mpu_init_struct)
{
    MPU->RNR = mpu_init_struct->region_number;
    MPU->RBAR = mpu_init_struct->region_base_address;
    MPU->RASR = ((uint32_t)mpu_init_struct->instruction_exec  << MPU_RASR_XN_Pos) |
                ((uint32_t)mpu_init_struct->access_permission << MPU_RASR_AP_Pos) |
                ((uint32_t)mpu_init_struct->tex_type          << MPU_RASR_TEX_Pos)|
                ((uint32_t)mpu_init_struct->access_shareable  << MPU_RASR_S_Pos)  |
                ((uint32_t)mpu_init_struct->access_cacheable  << MPU_RASR_C_Pos)  |
                ((uint32_t)mpu_init_struct->access_bufferable << MPU_RASR_B_Pos)  |
                ((uint32_t)mpu_init_struct->subregion_disable << MPU_RASR_SRD_Pos)|
                ((uint32_t)mpu_init_struct->region_size       << MPU_RASR_SIZE_Pos);
}

/*!
    \brief      enable the MPU region
    \param[in]  none
    \param[out] none
    \retval     none
*/
void mpu_region_enable(void)
{
    MPU->RASR |= MPU_RASR_ENABLE_Msk;
}

#endif /* __MPU_PRESENT */
