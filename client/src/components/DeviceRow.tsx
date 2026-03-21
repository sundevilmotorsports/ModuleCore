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
}

export function DeviceRow({ device }: DeviceRowProps) {
  async function handleIdent() {
    await invoke('identify', { canId: device.can_id })
  }

  return (
    <div className="flex items-center justify-between rounded-md border border-border bg-card px-4 py-3 transition-all duration-150 hover:border-border/80">
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
        onClick={handleIdent}
        className="font-mono text-[10px] tracking-[2px] text-muted-foreground hover:text-primary hover:border-primary"
      >
        IDENT
      </Button>
    </div>
  )
}
