import { invoke } from '@tauri-apps/api/core'
import { Button } from '@/components/ui/button'
//import { Badge } from '@/components/ui/badge'

export interface Device {
  can_id: number
  can_id_str: string
  module_type_str: string
  fw_version_str: string
}

interface DeviceRowProps {
  device: Device
  onSelect?: (device: Device) => void
  selected?: boolean
}

export function DeviceRow({ device, onSelect, selected = false }: DeviceRowProps) {
  async function handleIdent() {
    await invoke('identify', { canId: device.can_id })
  }

  function handleSelect() {
    if (onSelect) onSelect(device)
  }

  return (
    <div
      role="button"
      tabIndex={0}
      onClick={handleSelect}
      onKeyDown={(e) => { if (e.key === 'Enter') handleSelect() }}
      className={"flex items-center justify-between rounded-md border px-4 py-3 transition-all duration-150 hover:border-border/80 " + (selected ? 'border-primary bg-muted' : 'border-border bg-card')}
      style={{ cursor: onSelect ? 'pointer' : 'default' }}
    >
      <div className="flex items-center gap-6">
        <div className="flex flex-col gap-0.5">
          <span className="text-[9px] font-semibold tracking-[1.5px] text-muted-foreground">
            ID
          </span>
          <span className="font-mono text-[15px] font-semibold text-foreground">
            {device.can_id_str}
          </span>
        </div>

        <div className="flex flex-col gap-0.5">
          <span className="text-[9px] font-semibold tracking-[1.5px] text-muted-foreground">
            TYPE
          </span>
          <span className="font-mono text-[15px] text-muted-foreground">
            {device.module_type_str}
          </span>
        </div>

        <div className="flex flex-col gap-0.5">
          <span className="text-[9px] font-semibold tracking-[1.5px] text-muted-foreground">
            FW
          </span>
          <span className="font-mono text-[15px] text-muted-foreground">
            {device.fw_version_str}
          </span>
        </div>
      </div>

      <Button
        variant="outline"
        size="sm"
        onClick={(e) => { e.stopPropagation(); handleIdent() }}
        className="font-mono text-[10px] tracking-[2px] text-muted-foreground hover:text-primary hover:border-primary"
      >
        IDENT
      </Button>
    </div>
  )
}
