/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          zf_driver_gpio
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "fsl_iomuxc.h"
#include "zf_driver_gpio.h"

GPIO_Type * PORTPTR[] = GPIO_BASE_PTRS;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���Ÿ�������(�ڲ�����)
//  @return     void
//  Sample usage:           
//-------------------------------------------------------------------------------------------------------------------
void afio_init(uint32 muxRegister, uint32 muxMode, uint32 inputRegister, uint32 inputDaisy, uint32 configRegister, uint32 inputOnfield, uint32 pinconf)
{
    IOMUXC_SetPinMux(muxRegister, muxMode, inputRegister, inputDaisy, configRegister, inputOnfield);
    IOMUXC_SetPinConfig(muxRegister, muxMode, inputRegister, inputDaisy, configRegister, pinconf);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���Ÿ���ΪGPIO
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @param      pinconf     �������ã������ò�����zf_iomuxc.h�ļ���PINCONF_enumö��ֵȷ�����������ʹ�� | ���
//  @return     void
//  Sample usage:         
//-------------------------------------------------------------------------------------------------------------------
void gpio_iomuxc(gpio_pin_enum pin, uint32 pinconf)
{
    if((pin >= B0) && (pin <= C31))
    {
        *((volatile uint32_t *)(0x401F80BCU + 4 * (pin - B0))) = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5) | IOMUXC_SW_MUX_CTL_PAD_SION(0);
        *((volatile uint32_t *)(0x401F82ACU + 4 * (pin - B0))) = pinconf;
    }
    
    if((pin >= D0) && (pin <= D4))
    {
        *((volatile uint32_t *)(0x401F81D4U + 4 * (pin - D0))) = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5) | IOMUXC_SW_MUX_CTL_PAD_SION(0);
        *((volatile uint32_t *)(0x401F83C4U + 4 * (pin - D0))) = pinconf;
    }
    
    if((pin >= D12) && (pin <= D17))
    {
        *((volatile uint32_t *)(0x401F81BCU + 4 * (pin - D12))) = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5) | IOMUXC_SW_MUX_CTL_PAD_SION(0);
        *((volatile uint32_t *)(0x401F83ACU + 4 * (pin - D12))) = pinconf;
    }
    
    if((pin >= D26) && (pin <= D27))
    {
        *((volatile uint32_t *)(0x401F80B4U + 4 * (pin - D26))) = IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5) | IOMUXC_SW_MUX_CTL_PAD_SION(0);
        *((volatile uint32_t *)(0x401F82A4U + 4 * (pin - D26))) = pinconf;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO�������
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @param      dat         0���͵�ƽ 1���ߵ�ƽ
//  @return     void
//  Sample usage:           gpio_set(B9, 1);//B9 ����ߵ�ƽ
//-------------------------------------------------------------------------------------------------------------------
void gpio_set_level(gpio_pin_enum pin, uint8 dat)
{
    if(dat) GPIO_SetPinsOutput(PORTPTR[pin>>5],1<<(pin&0x1f));
    else    GPIO_ClearPinsOutput(PORTPTR[pin>>5],1<<(pin&0x1f));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO״̬��ȡ
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @return     uint8       0���͵�ƽ 1���ߵ�ƽ
//  Sample usage:           uint8 status = gpio_get(B9);//��ȡB9���ŵ�ƽ
//-------------------------------------------------------------------------------------------------------------------
uint8 gpio_get_level(gpio_pin_enum pin)
{
    return (GPIO_ReadPadStatus(PORTPTR[pin>>5],pin&0x1f));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO ��ת
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @return     void        
//  Sample usage:           gpio_toggle(B9);//B9���ŵ�ƽ��ת
//-------------------------------------------------------------------------------------------------------------------
void gpio_toggle_level(gpio_pin_enum pin)
{
    GPIO_PortToggle(PORTPTR[pin>>5],1<<(pin&0x1f));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO��������
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @param      dir         ���ŵķ���   �����GPO   ���룺GPI
//  @return     void        
//  Sample usage:           gpio_dir(B9, GPO);//����B9Ϊ���ģʽ
//-------------------------------------------------------------------------------------------------------------------
void gpio_set_dir(gpio_pin_enum pin, gpio_dir_enum dir, gpio_mode_enum pinconf)
{
    if (GPI == dir) PORTPTR[pin>>5]->GDIR &= ~(1U << (pin&0x1f));
    else            PORTPTR[pin>>5]->GDIR |= (1U << (pin&0x1f));
    gpio_iomuxc(pin, pinconf);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO��ʼ��
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @param      dir         ���ŵķ���   �����GPO   ���룺GPI
//  @param      dat         ���ų�ʼ��ʱ���õĵ�ƽ״̬�����ʱ��Ч 0���͵�ƽ 1���ߵ�ƽ
//  @param      pinconf     �������ã������ò�����zf_driver_gpio.h�ļ���gpio_mode_enumö��ֵȷ�����������ʹ�� | ���
//  @return     void
//  Sample usage:           gpio_init(B9, GPO, 1, GPO_PUSH_PULL);//B9��ʼ��ΪGPIO���ܡ����ģʽ���������
//-------------------------------------------------------------------------------------------------------------------
void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8 dat, uint32 pinconf)
{
    gpio_pin_config_t gpio_config;

    if(GPO == dir)      gpio_config.direction = kGPIO_DigitalOutput; 
    else if(GPI == dir) gpio_config.direction = kGPIO_DigitalInput;
    
    if(dat)     gpio_config.outputLogic =  1;
    else        gpio_config.outputLogic =  0;

    gpio_config.interruptMode = kGPIO_NoIntmode;
    gpio_iomuxc(pin, pinconf);

    GPIO_PinInit(PORTPTR[pin>>5], pin&0x1f, &gpio_config);
}






//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����GPIO�������
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @param      dat         0���͵�ƽ 1���ߵ�ƽ
//  @return     void
//  Sample usage:           fast_gpio_set(B9,1);//B9 ����ߵ�ƽ
//-------------------------------------------------------------------------------------------------------------------
void fast_gpio_set_level(gpio_pin_enum pin, uint8 dat)
{
    if(dat) GPIO_SetPinsOutput  (PORTPTR[(pin>>5) + 5],1<<(pin&0x1f));
    else    GPIO_ClearPinsOutput(PORTPTR[(pin>>5) + 5],1<<(pin&0x1f));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����GPIO�������
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @return     uint8       0���͵�ƽ 1���ߵ�ƽ
//  Sample usage:           uint8 status = fast_gpio_get(B9);//��ȡB9���ŵ�ƽ
//-------------------------------------------------------------------------------------------------------------------
uint8 fast_gpio_get_level(gpio_pin_enum pin)
{
    return (GPIO_ReadPinInput(PORTPTR[(pin>>5) + 5],pin&0x1f));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����GPIO ��ת
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @return     void        
//  Sample usage:           fast_gpio_toggle(B9);//B9���ŵ�ƽ��ת
//  @note                   ʹ�ñ��������з�תIO�����ֻ�ܴﵽ20��M��IO��ת���ʣ����ʹ��h�ļ��ļĴ����궨����Դﵽ���150M
//                          ������Ҫע�⣬���ʹ��while(1)+�궨��Ĵ�������תIO���ᷢ��Ƶ�����ֻ��100M��������Ϊwhile(1)Ҳռ����ָ��
//                          ��ﵽ���Ƶ�ʣ���Ҫ���������С�������ٶȣ�Ҳ����д�ܶ�궨��Ĵ�������תIO��������û��while(1)ָ����
//-------------------------------------------------------------------------------------------------------------------
void fast_gpio_toggle_level(gpio_pin_enum pin)
{
    GPIO_PortToggle(PORTPTR[(pin>>5) + 5],1<<(pin&0x1f));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����GPIO��������
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @param      dir         ���ŵķ���   �����GPO   ���룺GPI
//  @return     void        
//  Sample usage:           fast_gpio_dir(B9,GPO);//����B9Ϊ���ģʽ
//-------------------------------------------------------------------------------------------------------------------
void fast_gpio_set_dir(gpio_pin_enum pin, gpio_dir_enum dir, gpio_mode_enum pinconf)
{
    if (GPI == dir) PORTPTR[(pin>>5) + 5]->GDIR &= ~(1U << (pin&0x1f));
    else            PORTPTR[(pin>>5) + 5]->GDIR |= (1U << (pin&0x1f));
    gpio_iomuxc(pin, pinconf);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����GPIO��ʼ��
//  @param      pin         ѡ������� (��ѡ��Χ�� common.h ��gpio_pin_enumö��ֵȷ��)
//  @param      dir         ���ŵķ���   �����GPO   ���룺GPI
//  @param      dat         ���ų�ʼ��ʱ���õĵ�ƽ״̬�����ʱ��Ч 0���͵�ƽ 1���ߵ�ƽ
//  @param      pinconf     �������ã������ò�����zf_iomuxc.h�ļ���PINCONF_enumö��ֵȷ�����������ʹ�� | ���
//  @return     void
//  Sample usage:           fast_gpio_init(B9, GPO, 1, FAST_GPO_PUSH_PULL);//B9��ʼ��ΪGPIO���ܡ����ģʽ������ߵ�ƽ������47K IO�ٶ�2000MHZ ����ǿ��R0
//                          ����GPIO ��߿ɴ�150M�ķ�ת�������Ҫ�ﵽ����ٶȲ���ʹ�ÿ⺯���������ƽ����Ҫֱ�Ӳ����Ĵ������������ţ�������Բ鿴h�ļ����к궨��
//-------------------------------------------------------------------------------------------------------------------
void fast_gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8 dat, uint32 pinconf)
{
    uint16 temp_pin;
    gpio_iomuxc(pin, pinconf);
    
    switch(pin>>5)
    {
        case 1:
        {
            IOMUXC_GPR->GPR26 |= 1<<(pin&0x1f);
        }break;
        case 2:
        {
            IOMUXC_GPR->GPR27 |= 1<<(pin&0x1f);
        }break;
        case 3:
        {
            IOMUXC_GPR->GPR28 |= 1<<(pin&0x1f);
        }break;
    }
    
    temp_pin = (pin + 32*5);

    PORTPTR[temp_pin>>5]->IMR &= ~(1U << (temp_pin&0x1f));

    if (GPI == dir)
    {
        PORTPTR[temp_pin>>5]->GDIR &= ~(1U << (temp_pin&0x1f));
    }
    else
    {
        GPIO_PinWrite(PORTPTR[temp_pin>>5], (temp_pin&0x1f), dat);
        PORTPTR[temp_pin>>5]->GDIR |= (1U << (temp_pin&0x1f));
    }

    GPIO_SetPinInterruptConfig(PORTPTR[temp_pin>>5], (temp_pin&0x1f), kGPIO_NoIntmode);
}
