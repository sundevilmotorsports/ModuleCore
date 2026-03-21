import { useEffect, useState, useCallback } from 'react'
import { invoke } from '@tauri-apps/api/core'
import { listen } from '@tauri-apps/api/event'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Separator } from '@/components/ui/separator'
import { ScrollArea } from '@/components/ui/scroll-area'
import { DeviceRow, type Device } from '@/components/DeviceRow'

interface AppState {
  port_name: string
  devices: Device[]
}

export default function App() {
  const [state, setState] = useState<AppState>({ port_name: '', devices: [] })
  const [scanning, setScanning] = useState(false)

  const applyState = useCallback((s: AppState) => {
    setState(s)
  }, [])

  useEffect(() => {
    invoke<AppState>('get_state').then(applyState).catch(console.error)

    const unlisten = listen<AppState>('state-update', (event) => {
      applyState(event.payload)
    })

    return () => { unlisten.then(fn => fn()) }
  }, [applyState])

  async function handleDiscover() {
    setScanning(true)
    try {
      const s = await invoke<AppState>('get_state')
      applyState(s)
    } catch (e) {
      console.error(e)
    } finally {
      setScanning(false)
    }
  }

  const connected = state.port_name !== ''

  return (
    <div className="flex h-screen flex-col bg-background p-5 gap-3">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex flex-col gap-1">
          <span className="text-[11px] font-bold tracking-[3px] text-primary">
            DEVICES
          </span>
          <div className="flex items-center gap-2">
            <Badge
              variant={connected ? 'default' : 'secondary'}
              className="font-mono text-[10px] px-2 py-0"
            >
              {connected ? '● CONNECTED' : '○ SEARCHING'}
            </Badge>
            {connected && (
              <span className="text-[10px] text-muted-foreground font-mono">
                {state.port_name}
              </span>
            )}
          </div>
        </div>

        <Button
          variant="outline"
          size="sm"
          onClick={handleDiscover}
          disabled={scanning || !connected}
          className="font-mono text-[10px] tracking-[2px] text-muted-foreground hover:text-primary hover:border-primary"
        >
          {scanning ? 'SCANNING…' : 'DISCOVER'}
        </Button>
      </div>

      <Separator />

      {/* Device list */}
      <ScrollArea className="flex-1">
        <div className="flex flex-col gap-1.5 pr-3">
          {state.devices.length === 0 ? (
            <div className="flex items-center justify-center h-20 text-sm text-muted-foreground">
              {connected ? 'No devices found' : 'Waiting for device…'}
            </div>
          ) : (
            state.devices.map((dev) => (
              <DeviceRow key={dev.can_id} device={dev} />
            ))
          )}
        </div>
      </ScrollArea>
    </div>
  )
}
